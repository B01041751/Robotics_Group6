#!/usr/bin/env python3
import rospy
import numpy as np
import random
import os
import torch
import torch.nn as nn
import torch.optim as optim
from collections import deque
import threading
import time

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ContactsState, ModelStates 
from std_srvs.srv import Empty
from sound_play.libsoundplay import SoundClient

# --- DUELING DQN NETWORK ---
class DQN(nn.Module):
    def __init__(self, state_size, action_size):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(state_size, 128)
        self.fc2 = nn.Linear(128, 64)
        self.value_stream = nn.Linear(64, 1)
        self.advantage_stream = nn.Linear(64, action_size)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        v = self.value_stream(x)
        a = self.advantage_stream(x)
        return v + (a - a.mean(dim=1, keepdim=True))

class DQNAgent:
    def __init__(self):
        rospy.init_node('final_dqn_agent')

        # --- TUNNEL COORDINATES ---
        self.tunnel_x = 0.0
        self.tunnel_y_entry = 4.5  
        self.tunnel_y_exit = 7.8   # Robot must pass this Y to "unlock" gas reward
        self.passed_tunnel = False

        self.state_size = 5
        self.action_size = 3
        self.state = np.zeros(self.state_size)
        self.robot_x = self.robot_y = self.last_robot_y = 0.0
        
        self.stuck_counter = 0
        self.last_pose_y = 0.0
        self.collision = self.gas_hit = self.gas_near = False

        # RL Hyperparams
        self.gamma, self.lr = 0.95, 0.0005
        self.epsilon, self.epsilon_min, self.epsilon_decay = 1.0, 0.1, 0.996
        self.tau = 0.01

        self.memory = []
        self.priorities = []
        self.max_memory = 10000
        self.batch_size = 64
        self.alpha = 0.6

        self.device = torch.device("cpu")
        self.model = DQN(self.state_size, self.action_size).to(self.device)
        self.target_model = DQN(self.state_size, self.action_size).to(self.device)
        self.target_model.load_state_dict(self.model.state_dict())
        
        self.optimizer = optim.Adam(self.model.parameters(), lr=self.lr)
        self.criterion = nn.MSELoss()

        self.cmd_pub = rospy.Publisher('/group6bot_0/cmd_vel', Twist, queue_size=10)
        self.reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.sound = SoundClient()
        self.alarm_sound = "/home/ubuntu/com760cw2_group6/src/com760cw2_group6/sound/alarm.wav"

        rospy.Subscriber('/group6bot_0/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/gas_touch', ContactsState, self.gas_callback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.pose_callback)

        self.episode = 1
        self.total_reward = 0
        self.rate = rospy.Rate(10)

    def store(self, transition):
        if len(self.memory) >= self.max_memory:
            self.memory.pop(0)
            self.priorities.pop(0)
        self.memory.append(transition)
        self.priorities.append(max(self.priorities, default=1.0))

    def sample(self):
        probs = np.array(self.priorities) ** self.alpha
        probs /= probs.sum()
        indices = np.random.choice(len(self.memory), self.batch_size, p=probs)
        return [self.memory[i] for i in indices], indices

    def replay(self):
        if len(self.memory) < self.batch_size: return
        batch, indices = self.sample()
        states = torch.FloatTensor(np.array([b[0] for b in batch]))
        actions = torch.LongTensor(np.array([b[1] for b in batch]))
        rewards = torch.FloatTensor(np.array([b[2] for b in batch]))
        next_states = torch.FloatTensor(np.array([b[3] for b in batch]))
        dones = torch.FloatTensor(np.array([b[4] for b in batch]))

        current_q = self.model(states).gather(1, actions.unsqueeze(1)).squeeze()
        with torch.no_grad():
            next_actions = self.model(next_states).max(1)[1]
            next_q = self.target_model(next_states).gather(1, next_actions.unsqueeze(1)).squeeze()
            target_q = rewards + (self.gamma * next_q * (1 - dones))

        td_errors = torch.abs(target_q - current_q).detach().numpy()
        loss = self.criterion(current_q, target_q)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        for i, idx in enumerate(indices):
            self.priorities[idx] = td_errors[i] + 1e-5
        for t_param, l_param in zip(self.target_model.parameters(), self.model.parameters()):
            t_param.data.copy_(self.tau * l_param.data + (1.0 - self.tau) * t_param.data)

    def act(self, state):
        if random.random() < self.epsilon: return random.randint(0, 2)
        state_t = torch.FloatTensor(state).unsqueeze(0)
        return torch.argmax(self.model(state_t)).item()

    def scan_callback(self, msg):
        ranges = np.nan_to_num(np.array(msg.ranges), nan=3.0, posinf=3.0)
        regions = [np.min(ranges[i:i+72]) for i in range(0, 360, 72)]
        self.state = np.array(regions) / 3.0
        self.collision = True if np.min(regions) < 0.18 else False
        if np.min(regions) < 0.7:
            if not self.gas_near: self.sound.playWave(self.alarm_sound)
            self.gas_near = True
        else: self.gas_near = False

    def gas_callback(self, msg):
        if len(msg.states) > 0: self.gas_hit = True

    def pose_callback(self, msg):
        try:
            idx = msg.name.index("group6bot_0")
            self.robot_x = msg.pose[idx].position.x
            self.robot_y = msg.pose[idx].position.y
        except: pass

    def take_action(self, action):
        twist = Twist()
        front, left, right = self.state[2]*3, self.state[4]*3, self.state[0]*3
        
        # Stuck Recovery
        if abs(self.robot_y - self.last_pose_y) < 0.005: self.stuck_counter += 1
        else: self.stuck_counter = 0
        self.last_pose_y = self.robot_y

        if self.stuck_counter > 20:
            twist.linear.x, twist.angular.z = -0.2, 1.0
            self.cmd_pub.publish(twist)
            return

        # FORCE TUNNEL ALIGNMENT
        if not self.passed_tunnel and self.robot_y < self.tunnel_y_entry:
            # If the robot is drifting too far left or right of the tunnel mouth
            if abs(self.robot_x - self.tunnel_x) > 0.4:
                twist.linear.x = 0.15
                twist.angular.z = (self.tunnel_x - self.robot_x) * 1.5 
                self.cmd_pub.publish(twist)
                return

        # Side Overrides
        if left < 0.45: twist.linear.x, twist.angular.z = 0.15, -0.8
        elif right < 0.45: twist.linear.x, twist.angular.z = 0.15, 0.8
        else:
            if action == 0: twist.linear.x = 0.3
            elif action == 1: twist.linear.x, twist.angular.z = 0.05, 0.7
            else: twist.linear.x, twist.angular.z = 0.05, -0.7
        
        self.cmd_pub.publish(twist)

    def get_reward(self, action):
        if self.collision: return -500
        
        # LOGIC GATE: Update tunnel status
        if self.robot_y > self.tunnel_y_exit: 
            self.passed_tunnel = True

        # STAGED REWARD
        if not self.passed_tunnel:
            # STAGE 1: Robot MUST go to tunnel entrance
            if self.gas_hit: return -1000 # Massive penalty for skipping tunnel
            
            # Distance to tunnel mouth pull
            dist_to_entry = np.sqrt((self.robot_x - self.tunnel_x)**2 + (self.robot_y - self.tunnel_y_entry)**2)
            reward = 10.0 / (dist_to_entry + 0.1)
            
            # High reward for Y-Progress towards tunnel
            reward += (self.robot_y - self.last_robot_y) * 600.0
            
            # Penalty for wandering away from tunnel center on X-axis
            reward -= abs(self.robot_x - self.tunnel_x) * 20.0
        else:
            # STAGE 2: Robot has exited tunnel, now it can seek gas
            if self.gas_hit: return 10000 # The true jackpot
            reward = (self.robot_y - self.last_robot_y) * 400.0
            if action == 0: reward += 5.0

        if self.gas_near: reward -= 30.0
        return reward

    def print_dashboard(self, action, current_reward):
        os.system('clear')
        act_name = ["FORWARD", "LEFT-TURN", "RIGHT-TURN"][action]
        print(f"--- EPISODE {self.episode} | EPSILON {self.epsilon:.3f} ---")
        print(f"STAGE: {'HUNTING GAS' if self.passed_tunnel else 'TUNNEL APPROACH'}")
        print(f"POS: X:{self.robot_x:.2f} Y:{self.robot_y:.2f}")
        print(f"ACTION: {act_name} | REWARD: {current_reward:.2f}")
        print(f"SENSORS: L:{self.state[4]*3:.1f} F:{self.state[2]*3:.1f} R:{self.state[0]*3:.1f}")

    def reset(self):
        rospy.loginfo(f"Ep {self.episode} Over.")
        self.cmd_pub.publish(Twist())
        self.reset_sim()
        rospy.sleep(1.0)
        self.collision = self.gas_hit = self.passed_tunnel = False
        self.total_reward = self.stuck_counter = 0
        self.episode += 1
        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)

    def run(self):
        while not rospy.is_shutdown():
            prev_state = self.state.copy()
            self.last_robot_y = self.robot_y
            action = self.act(prev_state)
            self.take_action(action)
            
            rospy.sleep(0.1)
            reward = self.get_reward(action)
            self.total_reward += reward
            self.print_dashboard(action, reward)
            
            done = self.collision or self.gas_hit
            self.store((prev_state, action, reward, self.state.copy(), done))
            self.replay()
            if done: self.reset()

if __name__ == "__main__":
    try: DQNAgent().run()
    except rospy.ROSInterruptException: pass
