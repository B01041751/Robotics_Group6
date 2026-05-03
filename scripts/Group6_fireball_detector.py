#!/usr/bin/env python3
"""Listens to fireball contact sensors and reports when a robot is hit."""
import rospy
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import String

ROBOT_KEYWORDS = ['Group6Bot_0', 'Group6Bot_1']

hit_pub = None


def make_callback(fireball_name):
    def callback(msg):
        for state in msg.states:
            for collision in [state.collision1_name, state.collision2_name]:
                for robot in ROBOT_KEYWORDS:
                    if robot in collision:
                        hit = f"{fireball_name} hit {robot}"
                        rospy.logwarn("[Fireball] %s", hit)
                        hit_pub.publish(hit)
                        return
    return callback


if __name__ == '__main__':
    rospy.init_node('fireball_detector')
    hit_pub = rospy.Publisher('/fireball_hit', String, queue_size=10)

    rospy.Subscriber('/hazard_1/contact', ContactsState, make_callback('hazard_1'))
    rospy.Subscriber('/hazard_2/contact', ContactsState, make_callback('hazard_2'))

    rospy.loginfo("Fireball detector running — listening on /hazard_1/contact and /hazard_2/contact")
    rospy.spin()
