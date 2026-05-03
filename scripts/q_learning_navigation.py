#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import random
import pygame
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ContactsState

# --- Neural Network Architecture ---
class DQN(nn.Module):
    def __init__(self, state_size, action_size):
        super(DQN, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(state_size, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, action_size)
        )
    def forward(self, x):
        return self.fc(x)

class DQNNavNode:
    def __init__(self):
        rospy.init_node('dqn_hazard_nav')
        
        # Audio Setup
        pygame.mixer.init()
        pygame.mixer.music.load('/home/ubuntu/com760cw2_group6/src/com760cw2_group6/sound/beto.wav')
        pygame.mixer.music.play(-1) # Loop background music
        self.alarm = pygame.mixer.Sound('/home/ubuntu/com760cw2_group6/src/com760cw2_group6/sound/alarm.wav')

        # ROS Setup
        self.pub = rospy.Publisher('/Group6Bot_0/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/Group6Bot_0/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/Group6Bot_0/gas_touch', ContactsState, self.gas_callback)

        # Map Processing (DQN State)
        img_path = '/home/ubuntu/com760cw2_group6/src/com760cw2_group6/map_for_r1.png'
        self.map_img = cv2.imread(img_path, 0)
        self.grid_size = 20
        self.grid = cv2.resize(self.map_img, (self.grid_size, self.grid_size))
        
        # DQN Hyperparameters
        self.state_size = 2 # (x, y) coordinates
        self.action_size = 4 # Forward, Back, Left, Right
        self.model = DQN(self.state_size, self.action_size)
        self.optimizer = optim.Adam(self.model.parameters(), lr=0.001)
        self.memory = []
        
        self.current_pose = None
        self.gas_detected = False
        self.in_tunnel = False # Logic based on map coordinates

    def gas_callback(self, msg):
        if len(msg.states) > 0 and not self.gas_detected:
            rospy.loginfo("GAS DETECTED! SOUNDING ALARM!")
            self.alarm.play()
            self.gas_detected = True

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def get_reward(self, x, y):
        # Convert Gazebo coords to Grid coords for reward lookup
        gx = int(np.clip((x + 5) * 2, 0, 19))
        gy = int(np.clip((y + 5) * 2, 0, 19))
        
        reward = -1 # Default penalty for time spent
        
        # Tunnel Logic (Example coordinates: adjust to your tunnel location)
        if 5 < gx < 10 and 8 < gy < 12: 
            if not self.in_tunnel:
                reward = 5 # Entry reward
                self.in_tunnel = True
        elif self.in_tunnel:
            reward = 5 # Exit reward
            self.in_tunnel = False
            
        # Path following (White/Green pixels)
        if self.grid[19-gy, gx] > 127:
            reward += 2
            
        if self.gas_detected:
            reward += 10
            
        return reward

    def train_step(self):
        if len(self.memory) < 100: return
        
        batch = random.sample(self.memory, 32)
        for state, action, reward, next_state in batch:
            state_t = torch.FloatTensor(state)
            next_state_t = torch.FloatTensor(next_state)
            
            target = reward + 0.95 * torch.max(self.model(next_state_t)).item()
            output = self.model(state_t)[action]
            
            loss = nn.MSELoss()(output, torch.tensor(target))
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_pose:
                state = np.array([self.current_pose.position.x, self.current_pose.position.y])
                
                # Epsilon-greedy action
                if random.random() < 0.1:
                    action = random.randint(0, 3)
                else:
                    state_t = torch.FloatTensor(state)
                    action = torch.argmax(self.model(state_t)).item()
                
                # Execute action
                move = Twist()
                if action == 0: move.linear.x = 0.2
                elif action == 1: move.linear.x = -0.1
                elif action == 2: move.angular.z = 0.5
                elif action == 3: move.angular.z = -0.5
                self.pub.publish(move)
                
                # Get reward and store memory
                rospy.sleep(0.1)
                reward = self.get_reward(self.current_pose.position.x, self.current_pose.position.y)
                next_state = np.array([self.current_pose.position.x, self.current_pose.position.y])
                self.memory.append((state, action, reward, next_state))
                
                if len(self.memory) > 1000: self.memory.pop(0)
                self.train_step()
                
            rate.sleep()

if __name__ == '__main__':
    node = DQNNavNode()
    node.run()
