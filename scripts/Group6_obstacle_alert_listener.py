#!/usr/bin/env python3
# rospy is the main ROS Python library for nodes and communication
# rospy is the ROS Python library used to create nodes and communicate with ROS topics
# Point is used to represent a position or coordinate (x, y, z)
# Twist is used to send movement commands (linear and angular velocity) to the robot

import rospy
from geometry_msgs.msg import Point, Twist
# Math library is used for mathematical calculations such as distance or angle computation
import math

class ObstacleAlertListener:

    def __init__(self):
        # initialise ROS node
        rospy.init_node('obstacle_alert_listener')

        # publisher to control robot movement
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # subscribe to obstacle alerts coming from other robot
        rospy.Subscriber('/obstacle_alert', Point, self.callback)

    def callback(self, msg):

        # get obstacle position sent by Robot 1
        x = msg.x
        y = msg.y

        # calculate distance of obstacle from robot
        distance = math.sqrt(x**2 + y**2)

        rospy.logwarn(f"Robot0 received obstacle at (x={x:.2f}, y={y:.2f}), dist={distance:.2f}")

        # create velocity message
        twist = Twist()

        # if obstacle is close, take action
        if distance < 1.0:

            rospy.logwarn("Obstacle nearby → avoiding")

            # stop forward movement
            twist.linear.x = 0.0

            # decide turning direction based on obstacle position
            # if y > 0 → obstacle is on left → turn right
            # if y < 0 → obstacle is on right → turn left
            if y > 0:
                twist.angular.z = -1.0  # turn right
            else:
                twist.angular.z = 1.0   # turn left

        else:
            # if obstacle is far, just keep moving forward
            twist.linear.x = 0.2
            twist.angular.z = 0.0

        # publish movement command
        self.cmd_pub.publish(twist)


if __name__ == '__main__':
    try:
        # create object of class
        ObstacleAlertListener()

        # keep node running
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
