#!/usr/bin/env python3

import json
import rospy
import rospkg
import gymnasium as gym
import numpy as np
import os
import subprocess
from gymnasium import spaces
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor

class HazardAudioEnv(gym.Env):
    def __init__(self):
        super(HazardAudioEnv, self).__init__()
        # Initialize ROS Node
        try:
            rospy.init_node('hazard_audio_node', anonymous=True)
        except rospy.exceptions.ROSException:
            pass # Already initialized

        # 🎵 AUDIO PATHS
        _pkg = rospkg.RosPack().get_path('com760cw2_group6')
        self.bg_music = os.path.join(_pkg, 'sound', 'beto.wav')
        self.alarm_sound = os.path.join(_pkg, 'sound', 'alarm.wav')
        self.bg_process = None

        # 🤖 ROS INTERFACE
        self.pub_vel = rospy.Publisher('/group6bot_0/cmd_vel', Twist, queue_size=1)
        rospy.wait_for_service('/gazebo/reset_simulation')
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        rospy.wait_for_service('/gazebo/unpause_physics')
        self.unpause_proxy = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        
        # Subscribe to sensors
        rospy.Subscriber('/group6bot_0/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/group6bot_0/odom', Odometry, self.odom_callback)

        # ⚙️ SPACES
        # Action: [Linear Velocity (0.1 to 0.5), Angular Velocity (-1.0 to 1.0)]
        # Starting at 0.1 linear prevents the "frozen robot" issue during early training
        self.action_space = spaces.Box(
            low=np.array([0.1, -1.0], dtype=np.float32),
            high=np.array([0.5, 1.0], dtype=np.float32),
            dtype=np.float32
        )

        # Obs: [12 laser sectors, 12 laser deltas, X-pos, Y-pos, Waypoint Index]
        self.observation_space = spaces.Box(
            low=-20, high=20, shape=(27,), dtype=np.float32
        )

        # 📍 WAYPOINTS – final waypoint is read from layout.json so it tracks
        #   the randomised gas zone position written by generate_world.py.
        _layout_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'worlds', 'layout.json')
        try:
            with open(_layout_path) as _f:
                _layout = json.load(_f)
            _gas = _layout['gas']
            _gas_pos = np.array([_gas['x'], _gas['y']])
        except Exception as _e:
            rospy.logwarn(f'Could not read layout.json ({_e}); using default gas position')
            _gas_pos = np.array([2.0, -6.0])

        self.waypoints = [
            np.array([2.0, 0.0]),
            np.array([4.0, 0.0]),
            np.array([6.5, 0.0]),
            np.array([0.0, 4.0]),
            np.array([0.0, 7.0]),
            np.array([-9.5, 0.0]),
            _gas_pos,               # FINAL: GAS CLOUD (randomised each run)
        ]

        # INTERNAL STATE
        self.current_wp_idx = 0
        self.robot_pos = np.array([0.0, 0.0])
        self.laser_data = np.ones(12) * 10.0
        self.prev_laser_data = np.ones(12) * 10.0
        self.steps_in_episode = 0

    def laser_callback(self, data):
        self.prev_laser_data = np.copy(self.laser_data)
        num_sectors = 12
        ranges = np.array(data.ranges, dtype=np.float32)
        ranges = np.where(np.isfinite(ranges), ranges, 10.0)
        samples_per_sector = len(ranges) // num_sectors
        sectors = ranges[:samples_per_sector * num_sectors].reshape(num_sectors, samples_per_sector)
        self.laser_data = np.min(sectors, axis=1)

    def odom_callback(self, data):
        self.robot_pos = np.array([data.pose.pose.position.x, data.pose.pose.position.y])

    def get_obs(self):
        delta_laser = self.laser_data - self.prev_laser_data
        return np.concatenate([self.laser_data, delta_laser, self.robot_pos, [float(self.current_wp_idx)]]).astype(np.float32)

    def trigger_alarm(self):
        rospy.loginfo("🎯 MISSION SUCCESS: TARGET REACHED")
        self.stop_bg_music()
        self.pub_vel.publish(Twist()) 
        subprocess.Popen(["aplay", "-q", self.alarm_sound])

    def step(self, action):
        self.steps_in_episode += 1
        
        # 1. EXECUTE ACTION
        vel = Twist()
        vel.linear.x = float(action[0])
        vel.angular.z = float(action[1])
        self.pub_vel.publish(vel)
        rospy.sleep(0.05) # Control loop frequency

        # 2. EVALUATE STATE
        obs = self.get_obs()
        target_wp = self.waypoints[self.current_wp_idx]
        dist_to_wp = np.linalg.norm(self.robot_pos - target_wp)
        min_dist = np.min(self.laser_data)
        
        reward = 0.0
        done = False
        is_final_goal = (self.current_wp_idx == len(self.waypoints) - 1)

        # ❌ COLLISION CHECK (Grace period of 10 steps to allow spawn stability)
        if min_dist < 0.25 and self.steps_in_episode > 10:
            if not (is_final_goal and dist_to_wp < 1.3):
                self.stop_bg_music()
                return obs, -500.0, True, False, {}

        # 🚀 WAYPOINT PROGRESS
        success_radius = 1.3 if is_final_goal else 0.8
        if dist_to_wp < success_radius:
            self.current_wp_idx += 1
            reward += 2000.0
            rospy.loginfo(f"🚩 Waypoint {self.current_wp_idx} Cleared!")
            
            if self.current_wp_idx >= len(self.waypoints):
                self.trigger_alarm()
                return obs, 10000.0, True, False, {}

        # 🧭 REWARD SHAPING
        reward -= dist_to_wp * 0.5      # Penalty for distance
        reward += action[0] * 5.0       # Reward for moving forward
        reward -= abs(action[1]) * 0.1  # Slight penalty for excessive spinning

        # ⏱️ TIMEOUT
        if self.steps_in_episode > 1000:
            done = True

        return obs, float(reward), done, False, {}

    def play_bg_music(self):
        self.stop_bg_music()
        if os.path.exists(self.bg_music):
            self.bg_process = subprocess.Popen(["aplay", self.bg_music], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

    def stop_bg_music(self):
        if self.bg_process:
            try: self.bg_process.terminate()
            except: pass
            self.bg_process = None

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        # Stop and Reset Gazebo
        self.pub_vel.publish(Twist())
        try:
            self.reset_proxy()
            rospy.sleep(0.5)
            self.unpause_proxy()
        except: pass

        rospy.sleep(1.2) # Essential delay for physics to catch up
        self.current_wp_idx = 0
        self.steps_in_episode = 0
        self.play_bg_music()
        return self.get_obs(), {}

# --- MAIN TRAINING LOOP ---
if __name__ == "__main__":
    env = Monitor(HazardAudioEnv())
    
    # MLP Policy (Multi-Layer Perceptron) for vector-based sensor data
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        gamma=0.99,
        ent_coef=0.01,
        tensorboard_log="./ppo_logs/",
        device="cpu"
    )

    print("--- STARTING MISSION TRAINING ---")
    try:
        model.learn(total_timesteps=300000)
        model.save("ppo_hazard_final")
        print("--- MODEL SAVED ---")
    except KeyboardInterrupt:
        print("--- SAVING PROGRESS ---")
        model.save("ppo_hazard_interrupted")
