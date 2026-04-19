#!/usr/bin/env python3
import rospy
import rospkg
import gymnasium as gym
import numpy as np
import math
import tf
import cv2
import subprocess
import os
import signal
import sys
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ContactsState, ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty

class HazardWorldEnv(gym.Env):
    def __init__(self):
        super().__init__()
        self.ns = "/group6bot_0" 
        self.cmd_vel_pub = rospy.Publisher(f'{self.ns}/cmd_vel', Twist, queue_size=10)
        
        # Audio
        _pkg = rospkg.RosPack().get_path('com760cw2_group6')
        self.bg_music_path = os.path.join(_pkg, 'sound', 'beto.wav')
        self.alarm_path = os.path.join(_pkg, 'sound', 'alarm.wav')
        self.music_process = None
        self.alarm_played = False 

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        self.start_background_music()

        # Gazebo Services
        rospy.wait_for_service('/gazebo/reset_simulation')
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Subscribers
        rospy.Subscriber(f'{self.ns}/odom', Odometry, self.odom_callback)
        rospy.Subscriber(f'{self.ns}/scan', LaserScan, self.scan_callback)
        rospy.Subscriber(f'{self.ns}/camera/image_raw', Image, self.image_callback)
        rospy.Subscriber('/gas_touch', ContactsState, self.gas_callback)

        self.bridge = CvBridge()
        self.pose = None
        self.yaw = 0.0
        self.laser_sectors = np.ones(8) * 10.0
        self.fireball_visible = False
        self.fireball_area = 0
        self.in_gas = False
        self.prev_dist = 15.0
        
        self.current_state_idx = 0
        self.states = ['GO_TO_WALL', 'FIREBALL_ZONE', 'AVOID_TREE', 'TUNNEL_ENTRY', 'TUNNEL_EXIT', 'GAS_CLOUD']
        self.targets = {
            'GO_TO_WALL':    (2.5, 0.0),   
            'FIREBALL_ZONE': (5.5, 0.0),   
            'AVOID_TREE':    (8.0, 8.0),   
            'TUNNEL_ENTRY':  (0.0, 4.0),   
            'TUNNEL_EXIT':   (0.0, 9.5),   
            'GAS_CLOUD':     (2.5, -6.5)   
        }

        # Randomize once on first reset() call, not here, to avoid double randomization
        self.first_run = True

        self.action_space = gym.spaces.Box(
            low=np.array([0.1, -1.0], dtype=np.float32), 
            high=np.array([0.8, 1.0], dtype=np.float32), 
            dtype=np.float32
        )
        self.observation_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(11,), dtype=np.float32)

    def signal_handler(self, sig, frame):
        self.cleanup_audio()
        sys.exit(0)

    def cleanup_audio(self):
        if self.music_process:
            try:
                os.killpg(os.getpgid(self.music_process.pid), signal.SIGKILL)
                self.music_process.wait()
            except: pass
            self.music_process = None

    def randomize_targets(self):
        """Randomize waypoint positions while maintaining sequence structure"""
        # Define exploration zones for each checkpoint (keep them spread out)
        zones = {
            'GO_TO_WALL':    ((-1.0, 4.0), (-2.0, 2.0)),      # x, y ranges
            'FIREBALL_ZONE': ((3.0, 7.0), (-2.0, 2.0)),       # middle area
            'AVOID_TREE':    ((6.0, 9.5), (6.0, 9.5)),        # top-right quadrant
            'TUNNEL_ENTRY':  ((-2.0, 2.0), (2.0, 6.0)),       # left-center
            'TUNNEL_EXIT':   ((-2.0, 2.0), (8.0, 10.0)),      # top-left
            'GAS_CLOUD':     ((0.0, 4.0), (-8.0, -4.0))       # bottom (with mannequin)
        }
        
        for state_name, ((x_min, x_max), (y_min, y_max)) in zones.items():
            rx = np.random.uniform(x_min, x_max)
            ry = np.random.uniform(y_min, y_max)
            self.targets[state_name] = (rx, ry)
        
        # Move mannequin and gas zone together with a random yaw
        gas_x, gas_y = self.targets['GAS_CLOUD']
        self.move_mannequin(gas_x, gas_y, yaw=float(np.random.uniform(0.0, 2.0 * math.pi)))
        
        rospy.loginfo(f"🎲 Environment randomized: {self.targets}")

    def move_mannequin(self, x, y, yaw=0.0):
        """Move mannequin to stay with gas zone, combining 90-deg pitch with randomised yaw"""
        try:
            # Combine pitch=90deg (about X) and yaw (about Z) into one quaternion.
            # pitch quaternion: qx=sin(pi/4), qw=cos(pi/4)
            # yaw   quaternion: qz=sin(yaw/2), qw=cos(yaw/2)
            half_yaw = yaw / 2.0
            qx = math.cos(half_yaw) * 0.707
            qy = math.sin(half_yaw) * 0.707
            qz = math.sin(half_yaw) * 0.707
            qw = math.cos(half_yaw) * 0.707

            state = ModelState()
            state.model_name = 'gas_victim'
            state.pose.position.x = x
            state.pose.position.y = y
            state.pose.position.z = 0.05
            state.pose.orientation.x = qx
            state.pose.orientation.y = qy
            state.pose.orientation.z = qz
            state.pose.orientation.w = qw
            state.twist.linear.x = 0
            state.twist.linear.y = 0
            state.twist.linear.z = 0

            self.set_model_state(state)
            rospy.loginfo(f"💀 Mannequin moved to gas zone at ({x:.2f}, {y:.2f})")
        except Exception as e:
            rospy.logwarn(f"Could not move mannequin: {e}")

    def randomize_fireballs(self):
        """Randomize fireball spawn positions"""
        try:
            for i, fireball_name in enumerate(['hazard_rolling_1', 'hazard_rolling_2'], 1):
                # Random position within bounds
                x = np.random.uniform(-8.0, 8.0)
                y = np.random.uniform(-8.0, 6.0)
                
                state = ModelState()
                state.model_name = fireball_name
                state.pose.position.x = x
                state.pose.position.y = y
                state.pose.position.z = 0.5
                state.twist.linear.x = 0
                state.twist.linear.y = 0
                state.twist.linear.z = 0
                
                self.set_model_state(state)
            rospy.loginfo("🔥 Fireballs repositioned")
        except Exception as e:
            rospy.logwarn(f"Could not randomize fireballs: {e}")

    def randomize_robot_spawn(self):
        """Randomize robot spawn position"""
        try:
            # Robot spawns in safe starting zone (avoid gas and obstacles)
            x = np.random.uniform(-3.0, 1.0)
            y = np.random.uniform(-2.0, 2.0)
            
            state = ModelState()
            state.model_name = 'group6bot_0'
            state.pose.position.x = x
            state.pose.position.y = y
            state.pose.position.z = 0.1
            state.pose.orientation.w = 1.0
            state.twist.linear.x = 0
            state.twist.linear.y = 0
            state.twist.linear.z = 0
            
            self.set_model_state(state)
            rospy.loginfo(f"🤖 Robot spawned at ({x:.2f}, {y:.2f})")
        except Exception as e:
            rospy.logwarn(f"Could not randomize robot spawn: {e}")

    def start_background_music(self):
        if os.path.exists(self.bg_music_path):
            try:
                cmd = f"while :; do aplay -q '{self.bg_music_path}'; done"
                self.music_process = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)
            except: pass

    def play_alarm(self):
        if not self.alarm_played:
            try:
                subprocess.Popen(["aplay", "-q", self.alarm_path])
                self.alarm_played = True
            except: pass

    def odom_callback(self, msg):
        self.pose = msg.pose.pose
        q = (self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w)
        self.yaw = tf.transformations.euler_from_quaternion(q)[2]

    def scan_callback(self, msg):
        ranges = np.nan_to_num(np.array(msg.ranges), nan=10.0, posinf=10.0)
        sector_size = len(ranges) // 8
        if sector_size > 0:
            for i in range(8):
                self.laser_sectors[i] = np.min(ranges[i*sector_size:(i+1)*sector_size])

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array([0, 150, 150]), np.array([20, 255, 255]))
            self.fireball_area = np.sum(mask) / 255
            self.fireball_visible = self.fireball_area > 500
        except: pass

    def gas_callback(self, msg):
        if len(msg.states) > 0: self.in_gas = True

    def get_obs(self):
        if self.pose is None: return np.zeros(11, dtype=np.float32)
        state_key = self.states[self.current_state_idx]
        tx, ty = self.targets[state_key]
        px, py = self.pose.position.x, self.pose.position.y
        dist = math.hypot(tx - px, ty - py)
        angle_to_target = math.atan2(ty - py, tx - px)
        diff_angle = math.atan2(math.sin(angle_to_target - self.yaw), math.cos(angle_to_target - self.yaw))
        proximity_lasers = np.clip(1.0 - (self.laser_sectors / 4.0), 0, 1)
        return np.concatenate([proximity_lasers, [np.clip(dist/15.0, 0, 1)], [diff_angle/math.pi], [1.0 if self.fireball_visible else 0.0]]).astype(np.float32)

    def step(self, action):
        lin_v, ang_v = float(action[0]), float(action[1])
        min_laser = np.min(self.laser_sectors)
        
        if min_laser < 0.7 or (self.fireball_visible and self.fireball_area > 4000):
            lin_v = np.clip(lin_v, 0.1, 0.35) 
        
        move_cmd = Twist()
        move_cmd.linear.x, move_cmd.angular.z = lin_v, ang_v
        self.cmd_vel_pub.publish(move_cmd)
        
        rospy.sleep(0.1) 
        obs = self.get_obs()
        reward, done = 0, False
        dist_to_goal = obs[8] * 15.0
        current_target_name = self.states[self.current_state_idx]

        # --- SEQUENCE LOCK: penalise entering gas before reaching GAS_CLOUD waypoint ---
        if self.in_gas and current_target_name != 'GAS_CLOUD':
            print(f"!!! ILLEGAL GAS ENTRY: Resetting from {current_target_name} !!!")
            reward -= 5000000
            return obs, reward, True, False, {}

        # 1. COLLISION LOGIC
        if min_laser < 0.22:
            if current_target_name == 'GAS_CLOUD':
                self.play_alarm()
                reward += 500000000
                print(">>>> MISSION COMPLETE <<<<")
                return obs, reward, True, False, {}
            else:
                reward -= 2000000
                print(f"!!! CRASH RESET: {current_target_name} !!!")
                return obs, reward, True, False, {}

        # 2. CHECKPOINT SUCCESS
        if dist_to_goal < 1.0:
            if current_target_name == 'GAS_CLOUD':
                self.play_alarm()
                reward += 500000000
                return obs, reward, True, False, {}
            else:
                reward += 20000000
                self.current_state_idx += 1
                self.prev_dist = 15.0
                print(f"--- REACHED {current_target_name} ---")

        # 3. NAVIGATION REWARDS
        reward += (1.0 - abs(obs[9])) * 250.0 
        if dist_to_goal < self.prev_dist: reward += 1500.0
        else: reward -= 1600.0
        self.prev_dist = dist_to_goal

        return obs, reward, done, False, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        try: 
            self.cmd_vel_pub.publish(Twist())
            self.reset_proxy()
            rospy.sleep(0.5)
            
            # Randomize environment only on first run
            if self.first_run:
                self.randomize_targets()
                self.randomize_fireballs()
                self.randomize_robot_spawn()
                self.first_run = False
            
            self.unpause_proxy()
            rospy.sleep(1.2) 
        except Exception as e:
            rospy.logwarn(f"Reset error: {e}")
        
        self.current_state_idx = 0
        self.in_gas = False
        self.alarm_played = False
        self.prev_dist = 15.0
        return self.get_obs(), {}
