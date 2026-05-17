#!/usr/bin/env python3
import os
import sys
import math
import random
import signal
import subprocess
import threading
import time

import rospy
import rospkg
import numpy as np
import gymnasium as gym
import cv2
import tf

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ContactsState, ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty
from com760cw2_group6.srv import Group6StartBug2
from com760cw2_group6.msg import BotAlert


def _load_spawn_config():
    if os.environ.get('GAS1_X') is not None:
        return
    try:
        pkg_path = rospkg.RosPack().get_path('com760cw2_group6')
        env_file = os.path.join(pkg_path, 'worlds', 'spawn_config.env')
        with open(env_file) as f:
            for line in f:
                line = line.strip()
                if line.startswith('export '):
                    line = line[7:]
                if '=' in line and not line.startswith('#'):
                    key, _, val = line.partition('=')
                    os.environ.setdefault(key.strip(), val.strip())
        print(f"[HazardEnv] Loaded spawn_config.env from {env_file}")
    except Exception as e:
        print(f"[HazardEnv] WARNING: Could not load spawn_config.env: {e}")


class HazardWorldEnv(gym.Env):
    def __init__(self, randomise_gas=True, oscillate_fire=False):
        super().__init__()
        self.randomise_gas  = randomise_gas
        self.oscillate_fire = oscillate_fire
        _load_spawn_config()

        self.ns = "/Group6Bot_0"
        self.cmd_vel_pub  = rospy.Publisher(f'{self.ns}/cmd_vel', Twist, queue_size=10)
        self._alert_pub   = rospy.Publisher('/Group6Bot_0/alert', BotAlert, queue_size=10)
        rospy.Subscriber('/Group6Bot_1/alert', BotAlert, self._bot1_alert_cb)

        _pkg = rospkg.RosPack().get_path('com760cw2_group6')
        self.bg_music_path = os.path.join(_pkg, 'sound', 'beto.wav')
        self.alarm_path    = os.path.join(_pkg, 'sound', 'alarm.wav')
        self.music_process = None
        self._last_obstacle_alert_t = 0.0

        signal.signal(signal.SIGINT,  self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        self.start_background_music()

        rospy.wait_for_service('/gazebo/reset_simulation')
        self.reset_proxy      = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.unpause_proxy    = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state  = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Gazebo model names for the three gas clouds and their victim decorations
        self.gas_model_names  = ['gas_zone_1',   'gas_zone_2',   'gas_zone_3']
        self.gas_victim_names = ['gas_victim_1', 'gas_victim_2', 'gas_victim_3']

        # Bot 1 reset — restart Bug2 from its new spawn position after each Gazebo reset
        self._bot1_start_svc = None
        try:
            rospy.wait_for_service('/Group6Bot_1/start_bug2', timeout=5.0)
            self._bot1_start_svc = rospy.ServiceProxy('/Group6Bot_1/start_bug2', Group6StartBug2)
            rospy.loginfo("[Bot0 env] Bot1 start_bug2 service found — will reset on each episode")
        except rospy.ROSException:
            rospy.logwarn("[Bot0 env] /Group6Bot_1/start_bug2 not available — Bot1 will not be reset")

        self.laser_sectors    = np.ones(8) * 10.0
        self.last_scan_time      = None
        self.last_image_time     = None
        self.fireball_visible    = False
        self._prev_fireball_vis  = False
        self.fireball_area       = 0
        self.fireball_cx      = 0.0   # horizontal centre, -1=left, +1=right
        self.green_visible    = False
        self.green_area       = 0
        self.green_cx         = 0.0   # horizontal centre, -1=left, +1=right
        self.bridge           = CvBridge()
        self.pose             = None
        self.yaw              = 0.0
        self.roll             = 0.0
        self.pitch            = 0.0

        rospy.Subscriber(f'{self.ns}/odom',             Odometry,  self.odom_callback)
        rospy.Subscriber(f'{self.ns}/scan',             LaserScan, self.scan_callback)
        rospy.Subscriber(f'{self.ns}/camera/image_raw', Image,     self.image_callback)

        self.on_fire = False
        rospy.Subscriber('/fire_1/contact', ContactsState, self._fire_cb)
        rospy.Subscriber('/fire_2/contact', ContactsState, self._fire_cb)

        N_GAS = 3
        self.gas_positions = []
        for i in range(1, N_GAS + 1):
            gx = float(os.environ.get(f'GAS{i}_X', str(2.0 * i - 3)))
            gy = float(os.environ.get(f'GAS{i}_Y', '-5.0'))
            self.gas_positions.append((gx, gy))
        self.gas_visited   = [False] * N_GAS
        self.all_gas_found = False
        self.GAS_DETECT_RADIUS = 2.0

        home_x = float(os.environ.get('BOT0_X', '-8.5'))
        home_y = float(os.environ.get('BOT0_Y', '-1.5'))
        self.home = (home_x, home_y)

        # Contact-based gas detection disabled (Round 3): gas positions are randomised per
        # episode so fixed-world contact topics are unreliable. Proximity check handles finds.
        # for i in range(1, N_GAS + 1):
        #     rospy.Subscriber(f'/gas_{i}/contact', ContactsState,
        #                      lambda msg, idx=i-1: self._gas_contact_cb(msg, idx))

        self.gas_contact_flags = [False] * N_GAS
        self.in_gas           = False
        self.step_count       = 0
        self.stuck_steps      = 0
        self.MAX_STEPS        = 1500

        self._reward_breakdown = {
            'gas_find': 0.0, 'bearing': 0.0, 'camera':  0.0,
            'explore':  0.0, 'wall':    0.0, 'fire':    0.0,
            'reverse':  0.0, 'home':    0.0, 'step':    0.0,
            'terminal': 0.0,
        }
        self._fire_visible_count  = 0
        self._wall_proximity_count = 0

        # Exploration grid — 1m×1m cells across the 20m×20m arena
        self.GRID_CELL   = 1.0
        self.GRID_ORIGIN = -10.0
        self.visited_cells = set()

        # Fire positions — bot respawns if it gets within FIRE_DANGER_RADIUS metres
        self.FIRE_DANGER_RADIUS = 0.2
        self.fire_positions = []
        i = 1
        while True:
            fx = os.environ.get(f'FIRE{i}_X')
            fy = os.environ.get(f'FIRE{i}_Y')
            if fx is None or fy is None:
                break
            self.fire_positions.append((float(fx), float(fy)))
            i += 1
        if not self.fire_positions:
            self.fire_positions = [(-2.0, 1.5), (3.5, -3.0)]

        # Optional fire oscillation — circles of radius 0.5 m, 4 s period
        self._fire_osc_running = False
        self._fire_osc_thread  = None
        if self.oscillate_fire:
            self._fire_centers = list(self.fire_positions)
            self._fire_osc_running = True
            self._fire_osc_thread  = threading.Thread(
                target=self._fire_oscillation_loop, daemon=True
            )
            self._fire_osc_thread.start()

        self.action_space = gym.spaces.Box(
            low=np.array( [-0.4, -1.0], dtype=np.float32),
            high=np.array([ 0.8,  1.0], dtype=np.float32),
            dtype=np.float32,
        )
        # obs: 8 lidar + green_vis + green_cx + green_size + fire_vis + fire_cx + fire_size
        #      + frac_found + on_fire + home_heading + gas_bearing + gas_dist = 19
        self.observation_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(19,), dtype=np.float32
        )

        rospy.loginfo("[Bot0] === ENVIRONMENT LOADED ===")
        rospy.loginfo(f"[Bot0] Home: {self.home}")
        for i, (gx, gy) in enumerate(self.gas_positions):
            rospy.loginfo(f"[Bot0] Gas {i+1}: ({gx:.2f}, {gy:.2f})  detect_r={self.GAS_DETECT_RADIUS}m")
        for i, (fx, fy) in enumerate(self.fire_positions):
            rospy.loginfo(f"[Bot0] Fire {i+1}: ({fx:.2f}, {fy:.2f})  danger_r={self.FIRE_DANGER_RADIUS}m")
        rospy.loginfo("[Bot0] ==========================")

    # ------------------------------------------------------------------ #
    #  Audio                                                               #
    # ------------------------------------------------------------------ #

    def signal_handler(self, sig, frame):
        self.cleanup_audio()
        sys.exit(0)

    def cleanup_audio(self):
        if self.music_process:
            try:
                os.killpg(os.getpgid(self.music_process.pid), signal.SIGKILL)
            except Exception:
                pass
        subprocess.call(['pkill', '-9', 'aplay'], stderr=subprocess.DEVNULL)

    def start_background_music(self):
        if os.path.exists(self.bg_music_path):
            try:
                cmd = f"while :; do aplay -q {self.bg_music_path}; done"
                self.music_process = subprocess.Popen(
                    cmd, shell=True, preexec_fn=os.setsid
                )
            except Exception:
                pass

    def play_alarm(self):
        try:
            subprocess.Popen(['aplay', '-q', self.alarm_path])
        except Exception:
            pass

    # ------------------------------------------------------------------ #
    #  ROS callbacks                                                       #
    # ------------------------------------------------------------------ #

    def odom_callback(self, msg):
        self.pose = msg.pose.pose
        q = (self.pose.orientation.x, self.pose.orientation.y,
             self.pose.orientation.z, self.pose.orientation.w)
        self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion(q)

    def scan_callback(self, msg):
        self.last_scan_time = msg.header.stamp
        ranges = np.array(msg.ranges, dtype=np.float32)
        # Filter NaN, inf, and robot self-reads.
        # Rear wheel outer arcs reach 0.3386m from the LiDAR at (0.1, 0).
        # A 0.35m floor clears all body geometry with ~1cm margin.
        invalid = ~np.isfinite(ranges) | (ranges < 0.35)
        ranges[invalid] = 10.0
        ranges = np.clip(ranges, 0.0, 10.0)
        sector_size = len(ranges) // 8
        if sector_size > 0:
            for i in range(8):
                self.laser_sectors[i] = np.min(
                    ranges[i * sector_size:(i + 1) * sector_size]
                )

    def image_callback(self, msg):
        self.last_image_time = msg.header.stamp
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w   = cv_img.shape[:2]
            hsv    = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

            # Fireball (orange-red hue)
            fire_mask          = cv2.inRange(hsv, np.array([0, 80, 80]), np.array([20, 255, 255]))
            self.fireball_area    = int(np.sum(fire_mask) / 255)
            self.fireball_visible = self.fireball_area > 500
            if self.fireball_visible and not self._prev_fireball_vis and self.pose:
                nearest = min(self.fire_positions,
                              key=lambda f: math.hypot(self.pose.position.x - f[0], self.pose.position.y - f[1]))
                self._publish_alert(BotAlert.FIREBALL, nearest[0], nearest[1])
            self._prev_fireball_vis = self.fireball_visible
            if self.fireball_visible:
                M = cv2.moments(fire_mask)
                self.fireball_cx = ((M['m10'] / M['m00']) / w * 2.0 - 1.0) if M['m00'] > 0 else 0.0
            else:
                self.fireball_cx = 0.0

            # Gas cloud (green hue)
            green_mask      = cv2.inRange(hsv, np.array([40, 60, 60]), np.array([80, 255, 255]))
            self.green_area = int(np.sum(green_mask) / 255)
            self.green_visible = self.green_area > 300
            if self.green_visible:
                M = cv2.moments(green_mask)
                self.green_cx = ((M['m10'] / M['m00']) / w * 2.0 - 1.0) if M['m00'] > 0 else 0.0
            else:
                self.green_cx = 0.0

        except Exception:
            pass

    def _publish_alert(self, alert_type, x, y):
        msg = BotAlert()
        msg.bot_id     = 'bot0'
        msg.alert_type = alert_type
        msg.x          = float(x)
        msg.y          = float(y)
        self._alert_pub.publish(msg)

    _ALERT_NAMES = {BotAlert.GAS: 'GAS', BotAlert.FIREBALL: 'FIREBALL', BotAlert.OBSTACLE: 'OBSTACLE'}

    def _bot1_alert_cb(self, msg):
        label = self._ALERT_NAMES.get(msg.alert_type, str(msg.alert_type))
        rospy.loginfo(f"[Bot0] [Custom Message] from {msg.bot_id}: {label} at ({msg.x:.2f}, {msg.y:.2f})")

    def _gas_contact_cb(self, msg, idx):
        for state in msg.states:
            if 'Group6Bot_0' in state.collision1_name or \
               'Group6Bot_0' in state.collision2_name:
                if not self.gas_visited[idx]:
                    self.gas_contact_flags[idx] = True

    def _fire_cb(self, msg):
        for state in msg.states:
            if 'Group6Bot_0' in state.collision1_name or \
               'Group6Bot_0' in state.collision2_name:
                self.on_fire = True
                return

    # ------------------------------------------------------------------ #
    #  Gas randomisation (Round 3)                                        #
    # ------------------------------------------------------------------ #

    def _randomise_gas_positions(self):
        if not self.randomise_gas:
            return
        ARENA_MIN,    ARENA_MAX    = -8.5, 8.5
        MIN_FROM_HOME = 3.0
        MIN_FROM_FIRE = 2.5
        MIN_GAS_GAP   = 4.0

        new_positions = []
        for i in range(len(self.gas_positions)):
            placed = False
            for _ in range(200):
                x = random.uniform(ARENA_MIN, ARENA_MAX)
                y = random.uniform(ARENA_MIN, ARENA_MAX)
                if math.hypot(x - self.home[0], y - self.home[1]) < MIN_FROM_HOME:
                    continue
                if any(math.hypot(x - fx, y - fy) < MIN_FROM_FIRE
                       for fx, fy in self.fire_positions):
                    continue
                if any(math.hypot(x - px, y - py) < MIN_GAS_GAP
                       for px, py in new_positions):
                    continue
                new_positions.append((x, y))
                placed = True
                break
            if not placed:
                new_positions.append(self.gas_positions[i])
                rospy.logwarn(f"[Bot0] Gas {i+1}: no valid placement after 200 tries — keeping previous position")

        self.gas_positions = new_positions

        # Move Gazebo visual models to match the new Python positions.
        # gas_zone z=0.05 (flat cylinder), gas_victim z=0.05 with roll=1.57 (person upright).
        for i, (x, y) in enumerate(self.gas_positions):
            zone_state = ModelState()
            zone_state.model_name          = self.gas_model_names[i]
            zone_state.pose.position.x     = x
            zone_state.pose.position.y     = y
            zone_state.pose.position.z     = 0.05
            zone_state.pose.orientation.w  = 1.0
            zone_state.twist.linear.x      = 0.0
            zone_state.twist.linear.y      = 0.0
            zone_state.twist.linear.z      = 0.0
            zone_state.twist.angular.x     = 0.0
            zone_state.twist.angular.y     = 0.0
            zone_state.twist.angular.z     = 0.0
            zone_state.reference_frame     = 'world'

            victim_state = ModelState()
            victim_state.model_name          = self.gas_victim_names[i]
            victim_state.pose.position.x     = x
            victim_state.pose.position.y     = y
            victim_state.pose.position.z     = 0.05
            # roll=1.57 keeps the person mesh upright (world-file default orientation)
            victim_state.pose.orientation.x  = math.sin(1.57 / 2.0)
            victim_state.pose.orientation.w  = math.cos(1.57 / 2.0)
            victim_state.twist.linear.x      = 0.0
            victim_state.twist.linear.y      = 0.0
            victim_state.twist.linear.z      = 0.0
            victim_state.twist.angular.x     = 0.0
            victim_state.twist.angular.y     = 0.0
            victim_state.twist.angular.z     = 0.0
            victim_state.reference_frame     = 'world'

            try:
                self.set_model_state(zone_state)
                self.set_model_state(victim_state)
            except rospy.ServiceException as e:
                rospy.logwarn(f"[Bot0] Failed to move {self.gas_model_names[i]} to ({x:.2f},{y:.2f}): {e}")

        pos_str = "  ".join(f"G{i+1}=({x:.2f},{y:.2f})" for i, (x, y) in enumerate(self.gas_positions))
        rospy.loginfo(f"[Bot0] Gas randomised: {pos_str}")

    # ------------------------------------------------------------------ #
    #  Observation                                                         #
    # ------------------------------------------------------------------ #

    def _cell(self, px, py):
        cx = max(0, min(19, int((px - self.GRID_ORIGIN) / self.GRID_CELL)))
        cy = max(0, min(19, int((py - self.GRID_ORIGIN) / self.GRID_CELL)))
        return (cx, cy)

    def get_obs(self):
        if self.pose is None:
            return np.zeros(19, dtype=np.float32)

        px, py = self.pose.position.x, self.pose.position.y
        proximity  = np.clip(1.0 - (self.laser_sectors / 12.0), 0.0, 1.0)
        frac_found = sum(self.gas_visited) / len(self.gas_positions)

        green_size = min(self.green_area / 5000.0, 1.0)       # normalised 0→1
        fire_size  = min(self.fireball_area / 5000.0, 1.0)    # normalised 0→1

        # Heading toward home only once all gas found — zero otherwise
        if self.all_gas_found:
            angle_to_home = math.atan2(self.home[1] - py, self.home[0] - px)
            home_heading  = math.atan2(math.sin(angle_to_home - self.yaw),
                                       math.cos(angle_to_home - self.yaw)) / math.pi
        else:
            home_heading = 0.0

        # Bearing and distance to nearest unvisited gas — always-on dense signal
        unvisited = [(gx, gy) for i, (gx, gy) in enumerate(self.gas_positions)
                     if not self.gas_visited[i]]
        if unvisited and self.pose:
            nearest = min(unvisited, key=lambda g: math.hypot(px - g[0], py - g[1]))
            gx, gy  = nearest
            raw_angle   = math.atan2(gy - py, gx - px)
            gas_bearing = math.atan2(math.sin(raw_angle - self.yaw),
                                     math.cos(raw_angle - self.yaw)) / math.pi
            gas_dist_norm = min(math.hypot(px - gx, py - gy) / 20.0, 1.0)
        else:
            gas_bearing   = 0.0
            gas_dist_norm = 0.0

        return np.concatenate([
            proximity,                                        # 0-7:  lidar sectors
            [1.0 if self.green_visible    else 0.0],         # 8:    green visible (binary)
            [self.green_cx],                                  # 9:    green horiz centre (-1=left, +1=right)
            [green_size],                                     # 10:   green patch size (0→1, larger=closer)
            [1.0 if self.fireball_visible else 0.0],         # 11:   fireball visible (binary)
            [self.fireball_cx],                               # 12:   fireball horiz centre (-1 to +1)
            [fire_size],                                      # 13:   fireball size (0→1)
            [frac_found],                                     # 14:   fraction of gas found
            [1.0 if self.on_fire else 0.0],                  # 15:   fire contact
            [home_heading],                                   # 16:   heading to home (0 during search)
            [gas_bearing],                                    # 17:   bearing to nearest unvisited gas (-1 to +1)
            [gas_dist_norm],                                  # 18:   distance to nearest unvisited gas (0→1)
        ]).astype(np.float32)

    # ------------------------------------------------------------------ #
    #  Step                                                                #
    # ------------------------------------------------------------------ #

    def step(self, action):
        lin_v     = float(action[0])
        ang_v     = float(action[1])
        min_laser = np.min(self.laser_sectors)

        move_cmd = Twist()
        move_cmd.linear.x  = lin_v
        move_cmd.angular.z = ang_v
        self.cmd_vel_pub.publish(move_cmd)

        rospy.sleep(0.1)
        self.step_count += 1

        # Every 100 steps: log sensor data age so we can verify observations are fresh
        if self.step_count % 100 == 0:
            now = rospy.Time.now()
            scan_age  = (now - self.last_scan_time).to_sec()  if self.last_scan_time  else -1.0
            image_age = (now - self.last_image_time).to_sec() if self.last_image_time else -1.0
            rospy.loginfo(f"[Bot0] sensor sync — scan_age={scan_age:.3f}s  camera_age={image_age:.3f}s  step_dt=0.1s")
        obs          = self.get_obs()
        reward, done = 0.0, False

        px = self.pose.position.x if self.pose else 0.0
        py = self.pose.position.y if self.pose else 0.0

        # --- Exploration bonus: reward visiting new grid cells ---
        if self.pose:
            cell = self._cell(px, py)
            if cell not in self.visited_cells:
                self.visited_cells.add(cell)
                if not self.green_visible:  # don't reward exploring when gas is in sight
                    reward += 0.2
                    self._reward_breakdown['explore'] += 0.2

        # Contact-sensor gas finds disabled (Round 3) — proximity check below handles all finds.
        # for i in range(len(self.gas_positions)):
        #     if self.gas_contact_flags[i] and not self.gas_visited[i]:
        #         self.gas_visited[i] = True
        #         self.play_alarm()
        #         reward += 10.0
        #         self._reward_breakdown['gas_find'] += 10.0
        #         rospy.loginfo(f"[Bot0] Gas {i+1} found by contact ...")
        #     self.gas_contact_flags[i] = False

        # --- Proximity-based gas detection (distance check each step) ---
        prev_count = sum(self.gas_visited)
        for i, (gx, gy) in enumerate(self.gas_positions):
            if not self.gas_visited[i]:
                if self.pose and math.hypot(px - gx, py - gy) < self.GAS_DETECT_RADIUS:
                    self.gas_visited[i] = True
                    self.play_alarm()
                    reward += 10.0
                    self._reward_breakdown['gas_find'] += 10.0
                    rospy.loginfo(f"[Bot0] Gas {i+1} found ({sum(self.gas_visited)}/{len(self.gas_positions)})")
                    self._publish_alert(BotAlert.GAS, gx, gy)

        if all(self.gas_visited) and not self.all_gas_found:
            self.all_gas_found = True
            rospy.loginfo("[Bot0] All gas found — returning home")

        # Search-phase dense reward: bearing alignment (gated on clear forward space) + camera alignment
        if not self.all_gas_found and self.pose:
            gas_bearing_rad = obs[17] * math.pi
            # sectors 3 & 4 face forward (scan starts at -π rear, index 180 = 0 rad = ahead)
            if min(self.laser_sectors[3], self.laser_sectors[4]) > 0.6:
                _b = 0.2 * math.cos(gas_bearing_rad) * max(0.0, lin_v)
                reward += _b
                self._reward_breakdown['bearing'] += _b
        # Camera-alignment reward: only fires when actively approaching an unvisited gas zone.
        # Coefficient reduced from 0.3 → 0.1 to prevent it dominating the reward signal.
        # Distance gate prevents reward-hacking by parking next to a found gas.
        if not self.all_gas_found and self.green_visible and self.pose:
            nearest_unvisited_dist = obs[18] * 20.0  # gas_dist_norm × normaliser
            if nearest_unvisited_dist < 10.0:
                green_size = min(self.green_area / 5000.0, 1.0)
                cam_r = 0.1 * green_size * (1.0 - abs(self.green_cx))
                reward += cam_r
                self._reward_breakdown['camera'] += cam_r

        # --- Time limit ---
        if self.step_count >= self.MAX_STEPS:
            done = True
            rospy.loginfo(f"[Bot0] Time limit — gas {sum(self.gas_visited)}/{len(self.gas_positions)}")

        # --- Fire danger zone ---
        elif self.on_fire or (self.pose is not None and any(
            math.hypot(px - fx, py - fy) < self.FIRE_DANGER_RADIUS
            for fx, fy in self.fire_positions
        )):
            reward -= 10.0
            self._reward_breakdown['terminal'] -= 10.0
            done = True
            rospy.logwarn("[Bot0] Too close to fireball — episode ended")

        # --- Bot inverted / tipped over ---
        elif abs(self.roll) > 0.7 or abs(self.pitch) > 0.7:
            reward -= 10.0
            self._reward_breakdown['terminal'] -= 10.0
            done = True
            rospy.logwarn(f"[Bot0] Inverted (roll={self.roll:.2f} pitch={self.pitch:.2f}) — episode reset")

        # --- Wall proximity: escalating penalty (softened) ---
        elif min_laser < 0.45:
            proximity_frac = (0.45 - min_laser) / 0.45
            _w = -0.2 * proximity_frac * proximity_frac
            reward += _w
            self._reward_breakdown['wall'] += _w
            self._wall_proximity_count += 1
            now = time.time()
            if now - self._last_obstacle_alert_t > 5.0 and self.pose:
                self._publish_alert(BotAlert.OBSTACLE, self.pose.position.x, self.pose.position.y)
                self._last_obstacle_alert_t = now
            # Only count as stuck when also barely moving — wall-following is not stuck
            if abs(lin_v) + abs(ang_v) < 0.15:
                self.stuck_steps += 1
            else:
                self.stuck_steps = 0
            if self.stuck_steps >= 120:
                reward -= 10.0
                self._reward_breakdown['terminal'] -= 10.0
                done = True
                rospy.logwarn(f"[Bot0] Stuck too long")
        else:
            self.stuck_steps = 0

        # --- Mission complete ---
        if not done and self.all_gas_found and self.pose and \
                math.hypot(px - self.home[0], py - self.home[1]) < 2.0:
            reward += 40.0
            self._reward_breakdown['terminal'] += 40.0
            done = True
            rospy.loginfo("[Bot0] Mission complete — all gas found and returned home")

        # --- Navigation shaping (only on non-terminal steps) ---
        if not done:
            # Reversing is free only when genuinely escaping a wall (min_laser < 0.30).
            # Otherwise reverse motion is penalised, closing the oscillation loophole.
            if lin_v < 0 and min_laser > 0.30:
                reward -= 0.02
                self._reward_breakdown['reverse'] -= 0.02

            # Return phase: reward alignment toward home × forward velocity
            if self.all_gas_found and self.pose:
                home_heading_rad = obs[16] * math.pi
                _h = 0.1 * math.cos(home_heading_rad) * max(0.0, lin_v)
                reward += _h
                self._reward_breakdown['home'] += _h

            if self.fireball_visible:
                self._fire_visible_count += 1

            reward -= 0.005
            self._reward_breakdown['step'] -= 0.005

        if done:
            bd = self._reward_breakdown
            total = sum(bd.values())
            rospy.loginfo(
                f"[Bot0] EP_BREAKDOWN steps={self.step_count} total={total:+.1f} | "
                f"gas={bd['gas_find']:+.1f} bearing={bd['bearing']:+.1f} "
                f"camera={bd['camera']:+.1f} explore={bd['explore']:+.1f} "
                f"wall={bd['wall']:+.1f} fire={bd['fire']:+.1f} "
                f"reverse={bd['reverse']:+.1f} home={bd['home']:+.1f} "
                f"step={bd['step']:+.1f} terminal={bd['terminal']:+.1f} | "
                f"fire_vis={self._fire_visible_count}/{self.step_count} "
                f"wall_close={self._wall_proximity_count}/{self.step_count}"
            )

        return obs, reward, done, False, {}

    # ------------------------------------------------------------------ #
    #  Reset                                                               #
    # ------------------------------------------------------------------ #

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        try:
            self.cmd_vel_pub.publish(Twist())
            self.reset_proxy()
            self.unpause_proxy()
            rospy.sleep(1.2)
        except Exception:
            pass
        self._randomise_gas_positions()
        self.gas_visited       = [False] * len(self.gas_positions)
        self.gas_contact_flags = [False] * len(self.gas_positions)
        self.all_gas_found    = False
        self.on_fire          = False
        self.in_gas           = False
        self.green_visible    = False
        self.green_area       = 0
        self.green_cx         = 0.0
        self.fireball_visible = False
        self.fireball_area    = 0
        self.fireball_cx      = 0.0
        self.laser_sectors    = np.ones(8) * 10.0
        self.step_count       = 0
        self.stuck_steps      = 0
        self.visited_cells    = set()
        self._reward_breakdown     = {k: 0.0 for k in self._reward_breakdown}
        self._fire_visible_count   = 0
        self._wall_proximity_count = 0
        self.pose             = None
        self.yaw              = 0.0
        self.roll             = 0.0
        self.pitch            = 0.0

        # Wait for a valid pose before returning the first observation
        timeout = rospy.Time.now() + rospy.Duration(3.0)
        while self.pose is None and rospy.Time.now() < timeout:
            rospy.sleep(0.05)

        # Scan callbacks fire during the pose-wait and can re-populate laser_sectors
        # with stale pre-reset data.  Clear again so step() starts with a safe baseline.
        self.laser_sectors = np.ones(8) * 10.0
        self.on_fire       = False
        return self.get_obs(), {}

    # ------------------------------------------------------------------ #
    #  Fire oscillation                                                    #
    # ------------------------------------------------------------------ #

    def _fire_oscillation_loop(self):
        RADIUS = 0.5   # metres
        PERIOD = 4.0   # seconds per full rotation
        FIRE_Z = 0.55  # match world-file pose height
        t = 0.0
        dt = 0.05      # 20 Hz
        while self._fire_osc_running and not rospy.is_shutdown():
            t += dt
            for i, (cx, cy) in enumerate(self._fire_centers):
                angle = 2.0 * math.pi * t / PERIOD
                x = cx + RADIUS * math.cos(angle)
                y = cy + RADIUS * math.sin(angle)
                self.fire_positions[i] = (x, y)
                state = ModelState()
                state.model_name         = f'fire_{i + 1}'
                state.pose.position.x    = x
                state.pose.position.y    = y
                state.pose.position.z    = FIRE_Z
                state.pose.orientation.w = 1.0
                state.reference_frame    = 'world'
                try:
                    self.set_model_state(state)
                except Exception:
                    pass
            time.sleep(dt)

    def close(self):
        self._fire_osc_running = False
        self.cleanup_audio()
        super().close()

    def __del__(self):
        self.cleanup_audio()
