#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def move_fireballs():
    rospy.init_node('fireball_mover')
    pub1 = rospy.Publisher('/hazard_1/cmd_vel', Twist, queue_size=1)
    pub2 = rospy.Publisher('/hazard_2/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10)

    t = Twist()
    t.linear.x = 0.5  # forward speed
    t.angular.z = 0.2  # slow spin

    while not rospy.is_shutdown():
        pub1.publish(t)
        pub2.publish(t)
        rate.sleep()

if __name__ == '__main__':
    move_fireballs()
