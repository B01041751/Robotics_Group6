#!/usr/bin/env python3

import rospy
from com760cw2_group6.srv import SetBugStatus, SetBugStatusResponse
from geometry_msgs.msg import Twist

class BugController:

    def __init__(self):
        rospy.init_node('bug_service_server')

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.active = False
        self.speed = 0.2
        self.direction = "left"

        rospy.Service('set_bug_status', SetBugStatus, self.callback)

    def callback(self, req):
        self.active = req.flag
        self.speed = req.speed
        self.direction = req.direction

        rospy.loginfo(f"Bug behaviour: {self.active}, speed={self.speed}, dir={self.direction}")

        return SetBugStatusResponse("Service updated successfully")

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            twist = Twist()

            if self.active:
                twist.linear.x = self.speed

                if self.direction == "left":
                    twist.angular.z = 0.5
                elif self.direction == "right":
                    twist.angular.z = -0.5

            self.pub.publish(twist)
            rate.sleep()


if __name__ == '__main__':
    node = BugController()
    node.run()
