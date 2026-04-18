#!/usr/bin/env python3

import rospy
from com760cw2_group6.msg import QLearning

def publisher():
    pub = rospy.Publisher('q_learning', QLearning, queue_size=10)
    rospy.init_node('q_learning_publisher', anonymous=True)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = QLearning()
        msg.state = 1
        msg.action = 2
        msg.reward = 0.5
        msg.done = False

        pub.publish(msg)
        rospy.loginfo("Published QLearning message")

        rate.sleep()

if __name__ == '__main__':
    publisher()
