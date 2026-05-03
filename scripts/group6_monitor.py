#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Twist

class Group6Monitor:
    def __init__(self):
        rospy.init_node('group6_monitor', anonymous=True)

        # Subscribe to the robot's data
        rospy.Subscriber('/group6bot_0/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/group6bot_0/scan', LaserScan, self.scan_cb)
        rospy.Subscriber('/gas_touch', ContactsState, self.gas_cb)
        
        # Subscribe to the Hazard (Fireball) commands to see their speed
        rospy.Subscriber('/hazard_1/cmd_vel', Twist, self.h1_cb)

        self.robot_pos = (0, 0)
        self.min_dist = 0.0
        self.gas_status = "SAFE"
        self.h1_speed = 0.0

    def odom_cb(self, msg):
        self.robot_pos = (round(msg.pose.pose.position.x, 2), round(msg.pose.pose.position.y, 2))

    def scan_cb(self, msg):
        # Find the closest object recorded by LiDAR
        self.min_dist = round(min(msg.ranges), 2)

    def gas_cb(self, msg):
        self.gas_status = "DANGER: IN GAS" if len(msg.states) > 0 else "SAFE"

    def h1_cb(self, msg):
        self.h1_speed = round(msg.linear.x, 2)

    def log_status(self):
        # Clear terminal and print status
        print("\033[H\033[J") # ANSI escape codes to clear screen
        print("--- GROUP 6 MISSION MONITOR ---")
        print(f"Robot Position:  X: {self.robot_pos[0]}, Y: {self.robot_pos[1]}")
        print(f"Closest Obstacle: {self.min_dist} meters")
        print(f"Fireball 1 Speed: {self.h1_speed} m/s")
        print(f"Gas Zone Status:  {self.gas_status}")
        print("-------------------------------")

if __name__ == '__main__':
    monitor = Group6Monitor()
    rate = rospy.Rate(2) # Update twice per second
    while not rospy.is_shutdown():
        monitor.log_status()
        rate.sleep()
