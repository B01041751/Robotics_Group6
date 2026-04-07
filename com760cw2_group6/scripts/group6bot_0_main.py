#!/usr/bin/env python3
import rospy
import gym
import numpy as np
import math
import tf
import cv2
import subprocess
import os
import signal
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ContactsState
from std_srvs.srv import Empty

class HazardWorldEnv(gym.Env):
    def __init__(self):
        super().__init__()
        self.ns = "/group6bot_0" 
        self.cmd_vel_pub = rospy.Publisher(f'{self.ns}/cmd_vel', Twist, queue_size=10)
        
        # --- AUDIO PATHS ---
        self.bg_music_path = "/home/ubuntu/com760cw2_group6/src/com760cw2_group6/sound/beto.wav"
        self.alarm_path = "/home/ubuntu/com760cw2_group6/src/com760cw2_group6/sound/alarm.wav"
        self.music_process = None
        self.alarm_played = False 

        # Start Background Music
        self.start_background_music()

        # Services
        rospy.loginfo("Waiting for Gazebo services...")
        rospy.wait_for_service('/gazebo/reset_simulation')
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

        # ✅ FORCE PHYSICS START: This kicks Gazebo into gear if it starts paused
        try:
            self.unpause_proxy()
        except:
            pass

        # Subscribers
        rospy.Subscriber(f'{self.ns}/odom', Odometry, self.odom_callback)
        rospy.Subscriber(f'{self.ns}/scan', LaserScan, self.scan_callback)
        rospy.Subscriber(f'{self.ns}/camera/image_raw', Image, self.image_callback)
        rospy.Subscriber(f'{self.ns}/gas_touch', ContactsState, self.gas_callback)

        self.bridge = CvBridge()
        self.pose = None
        self.yaw = 0.0
        self.laser_sectors = np.ones(8) * 10.0
        self.fireball_visible = False
        self.in_gas = False
        self.current_state_idx = 0
        self.states = ['TUNNEL_DEBRIS', 'AVOID_TREE', 'FIREBALL_DODGE', 'GAS_ZONE']
        self.targets = {'TUNNEL_DEBRIS': (0.0, 7.0), 'AVOID_TREE': (8.0, 8.0), 'FIREBALL_DODGE': (5.0, 0.0), 'GAS_ZONE': (2.0, -6.0)}

        self.action_space = gym.spaces.Box(low=np.array([0.0, -1.5]), high=np.array([0.5, 1.5]), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(11,), dtype=np.float32)

    def start_background_music(self):
        try:
            if os.path.exists(self.bg_music_path):
                # Loop the background music so it doesn't stop during long training
                self.music_process = subprocess.Popen(
                    f"while :; do aplay -q {self.bg_music_path}; done", 
                    shell=True, preexec_fn=os.setsid
                )
                rospy.loginfo("Background music started (looping).")
            else:
                rospy.logwarn(f"Music file not found: {self.bg_music_path}")
        except Exception as e:
            rospy.logerr(f"Failed to start music: {e}")

    def play_alarm(self):
        if not self.alarm_played:
            try:
                subprocess.Popen(["aplay", "-q", self.alarm_path])
                rospy.loginfo("!!! ALARM: GAS CLOUD REACHED !!!")
                self.alarm_played = True
            except Exception as e:
                rospy.logerr(f"Failed to play alarm: {e}")

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
            mask = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([20, 255, 255]))
            self.fireball_visible = np.sum(mask) > 10000
        except: pass

    def gas_callback(self, msg):
        if len(msg.states) > 0:
            self.in_gas = True

    def get_obs(self):
        # ✅ BLOCKING CHECK: Ensure we don't return zeros while waiting for Odom
        while self.pose is None and not rospy.is_shutdown():
            rospy.loginfo_throttle(2, "Waiting for initial Odometry...")
            rospy.sleep(0.1)
            
        state_key = self.states[self.current_state_idx]
        tx, ty = self.targets[state_key]
        dist = math.hypot(tx - self.pose.position.x, ty - self.pose.position.y)
        angle = math.atan2(ty - self.pose.position.y, tx - self.pose.position.x) - self.yaw
        angle = math.atan2(math.sin(angle), math.cos(angle))
        
        return np.concatenate([
            np.clip(self.laser_sectors / 10.0, 0, 1), 
            [np.clip(dist / 15.0, 0, 1)], 
            [angle / math.pi], 
            [1.0 if self.fireball_visible else 0.0]
        ]).astype(np.float32)

    def step(self, action):
        move_cmd = Twist()
        move_cmd.linear.x = float(action[0])
        move_cmd.angular.z = float(action[1])
        self.cmd_vel_pub.publish(move_cmd)
        
        # Give Gazebo time to simulate the move
        rospy.sleep(0.1)

        obs = self.get_obs()
        reward = 0
        done = False
        dist_to_goal = obs[8] * 15.0

        if self.in_gas:
            self.play_alarm()
            reward += 1000
            done = True

        if np.min(self.laser_sectors) < 0.35: # Tightened collision box
            reward -= 200
            done = True

        if dist_to_goal < 0.8:
            reward += 500
            if self.current_state_idx < len(self.states) - 1:
                self.current_state_idx += 1
            else:
                done = True

        return obs, reward, done, {}

    def reset(self):
        # Stop robot before reset
        self.cmd_vel_pub.publish(Twist())
        try:
            self.reset_proxy()
            rospy.sleep(0.2)
            self.unpause_proxy()
            rospy.sleep(0.5)
        except: pass
        
        self.current_state_idx = 0
        self.in_gas = False
        self.alarm_played = False 
        return self.get_obs()

    def __del__(self):
        if self.music_process:
            os.killpg(os.getpgid(self.music_process.pid), signal.SIGTERM)
