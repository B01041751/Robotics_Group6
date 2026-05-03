#!/usr/bin/env python3

import rospy
import numpy as np
import random
import os
import torch
import torch.nn as nn
import torch.optim as optim
from collections import deque

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ContactsState
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty
from sound_play.libsoundplay import SoundClient


# =============================
# DQN NETWORK
# =============================
class DQN(nn.Module):
    def __init__(self, state_size, action_size):
        super(DQN, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(state_size, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, action_size)
        )

    def forward(self, x):
        return self.net(x)


# =============================
# AGENT
# =============================
class DQNAgent:

    def __init__(self):
        rospy.init_node('final_dqn_agent')

        self.mode = rospy.get_param("~mode", "train")

        # -----------------------------
        # PARAMETERS
        # -----------------------------
        self.state_size = 5  # 5 lidar regions
        self.action_size = 3

        self.gamma = 0.95
        self.lr = 0.001

        self.epsilon = 1.0 if self.mode == "train" else 0.0
        self.epsilon_min = 0.05
        self.epsilon_decay = 0.995

        self.batch_size = 64
        self.memory = deque(maxlen=5000)

        # -----------------------------
        # NETWORKS
        # -----------------------------
        self.device = torch.device("cpu")

        self.model = DQN(self.state_size, self.action_size).to(self.device)
        self.target_model = DQN(self.state_size, self.action_size).to(self.device)

        self.optimizer = optim.Adam(self.model.parameters(), lr=self.lr)

        self.update_target()

        # Load model
        if os.path.exists("dqn_model.pth"):
            self.model.load_state_dict(torch.load("dqn_model.pth"))
            rospy.loginfo("Loaded trained model")

        # -----------------------------
        # SOUND
        # -----------------------------
        self.sound = SoundClient()
        rospy.sleep(1)

        self.bg_sound = "/home/ubuntu/com760cw2_group6/src/com760cw2_group6/sound/beto.wav"
        self.alarm_sound = "/home/ubuntu/com760cw2_group6/src/com760cw2_group6/sound/alarm.wav"

        self.alarm_playing = False
        self.sound.playWave(self.bg_sound)
        rospy.Timer(rospy.Duration(30), lambda e: self.sound.playWave(self.bg_sound))

        # -----------------------------
        # ROS
        # -----------------------------
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/gas_touch', ContactsState, self.gas_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        self.reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        # -----------------------------
        # STATE
        # -----------------------------
        self.state = np.zeros(self.state_size)
        self.prev_state = np.zeros(self.state_size)

        self.collision = False
        self.gas_hit = False

        self.total_reward = 0
        self.episode = 0

        self.map_data = None
        self.visited = set()

        self.rate = rospy.Rate(10)

    # =============================
    # UPDATE TARGET NETWORK
    # =============================
    def update_target(self):
        self.target_model.load_state_dict(self.model.state_dict())

    # =============================
    # SCAN → STATE
    # =============================
    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)

        regions = [
            np.min(ranges[0:72]),
            np.min(ranges[72:144]),
            np.min(ranges[144:216]),
            np.min(ranges[216:288]),
            np.min(ranges[288:360])
        ]

        regions = [min(r, 3.0) for r in regions]

        self.state = np.array(regions) / 3.0

        if regions[2] < 0.2:
            self.collision = True

    def map_callback(self, msg):
        self.map_data = msg

    def exploration_bonus(self):
        if self.map_data is None:
            return 0

        explored = sum(1 for c in self.map_data.data if c != -1)

        if explored not in self.visited:
            self.visited.add(explored)
            return 5
        return 0

    def gas_callback(self, msg):
        if len(msg.states) > 0:
            self.gas_hit = True
            if not self.alarm_playing:
                self.sound.playWave(self.alarm_sound)
                self.alarm_playing = True
        else:
            self.alarm_playing = False

    # =============================
    # ACTION
    # =============================
    def act(self, state):
        if random.random() < self.epsilon:
            return random.randrange(self.action_size)

        state_t = torch.FloatTensor(state).unsqueeze(0)
        q_values = self.model(state_t)
        return torch.argmax(q_values).item()

    def take_action(self, action):
        twist = Twist()

        if action == 0:
            twist.linear.x = 0.3
        elif action == 1:
            twist.linear.x = 0.1
            twist.angular.z = 0.5
        elif action == 2:
            twist.linear.x = 0.1
            twist.angular.z = -0.5

        self.cmd_pub.publish(twist)

    # =============================
    # REWARD
    # =============================
    def get_reward(self, action):
        reward = 0

        if self.collision:
            reward -= 100

        if self.gas_hit:
            reward -= 50

        if action == 0:
            reward += 2
        else:
            reward -= 0.5

        reward += 1
        reward += self.exploration_bonus()

        return reward

    # =============================
    # MEMORY
    # =============================
    def remember(self, s, a, r, s2):
        self.memory.append((s, a, r, s2))

    # =============================
    # TRAIN
    # =============================
    def replay(self):
        if len(self.memory) < self.batch_size:
            return

        batch = random.sample(self.memory, self.batch_size)

        for s, a, r, s2 in batch:
            s = torch.FloatTensor(s)
            s2 = torch.FloatTensor(s2)

            target = r + self.gamma * torch.max(self.target_model(s2)).item()

            current = self.model(s)[a]

            loss = (current - target) ** 2

            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()

    # =============================
    # RESET
    # =============================
    def reset(self):
        rospy.loginfo(f"Episode {self.episode} Reward: {self.total_reward}")

        try:
            self.reset_sim()
        except:
            pass

        self.total_reward = 0
        self.collision = False
        self.gas_hit = False

        if self.mode == "train":
            self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)

        self.episode += 1

        if self.episode % 10 == 0:
            self.update_target()
            torch.save(self.model.state_dict(), "dqn_model.pth")

    # =============================
    # MAIN LOOP
    # =============================
    def run(self):

        while not rospy.is_shutdown():

            self.prev_state = self.state.copy()

            action = self.act(self.state)
            self.take_action(action)

            rospy.sleep(0.1)

            reward = self.get_reward(action)

            if self.mode == "train":
                self.remember(self.prev_state, action, reward, self.state)
                self.replay()

            self.total_reward += reward

            if self.collision or self.gas_hit:
                self.reset()

            self.collision = False
            self.gas_hit = False

            self.rate.sleep()


if __name__ == "__main__":
    agent = DQNAgent()
    agent.run()
