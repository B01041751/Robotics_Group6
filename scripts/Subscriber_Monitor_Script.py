#!/usr/bin/env python3
import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ContactsState, ModelStates

class AgentMonitor:
    def __init__(self):
        rospy.init_node('agent_monitor')

        # State tracking
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.last_y = 0.0

        self.front = self.left = self.right = 0.0

        self.collision = False
        self.gas_contact = False

        # Subscribers
        rospy.Subscriber('/group6bot_0/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/group6bot_0/cmd_vel', Twist, self.cmd_callback)
        rospy.Subscriber('/gas_touch', ContactsState, self.gas_callback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.pose_callback)

        self.rate = rospy.Rate(5)

    # --- LASER SCAN ---
    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=3.0, posinf=3.0)

        # Restore 5-sector logic
        sectors = [np.min(ranges[i:i+72]) for i in range(0, 360, 72)]

        self.left  = sectors[4]
        self.front = sectors[2]
        self.right = sectors[0]

        if np.min(sectors) < 0.18:
            self.collision = True
        else:
            self.collision = False

    # --- VELOCITY COMMAND ---
    def cmd_callback(self, msg):
        self.linear = msg.linear.x
        self.angular = msg.angular.z

    # --- GAS CONTACT ---
    def gas_callback(self, msg):
        self.gas_contact = len(msg.states) > 0

    # --- POSE ---
    def pose_callback(self, msg):
        try:
            idx = msg.name.index("group6bot_0")
            self.robot_x = msg.pose[idx].position.x
            self.robot_y = msg.pose[idx].position.y
        except:
            pass

    # --- DEBUG PRINT ---
    def display(self):
        rospy.loginfo(
            f"""
--- ROBOT DEBUG ---
Position: X={self.robot_x:.2f}, Y={self.robot_y:.2f}
Movement: ΔY={self.robot_y - self.last_y:.3f}

Laser:
  Left:  {self.left:.2f}
  Front: {self.front:.2f}
  Right: {self.right:.2f}

Cmd Vel:
  Linear:  {getattr(self, 'linear', 0):.2f}
  Angular: {getattr(self, 'angular', 0):.2f}

Flags:
  Collision: {self.collision}
  Gas Contact: {self.gas_contact}
---------------------
"""
        )
        self.last_y = self.robot_y

    def run(self):
        while not rospy.is_shutdown():
            self.display()
            self.rate.sleep()


if __name__ == "__main__":
    try:
        AgentMonitor().run()
    except rospy.ROSInterruptException:
        pass
