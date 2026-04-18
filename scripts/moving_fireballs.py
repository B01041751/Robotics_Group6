#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist

def move_fireballs():
    rospy.init_node('fireball_mover')
    pub1 = rospy.Publisher('/hazard_1/cmd_vel', Twist, queue_size=1)
    pub2 = rospy.Publisher('/hazard_2/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10)

    # Randomized velocities for each fireball
    linear_speed_1 = np.random.uniform(0.3, 0.7)
    angular_speed_1 = np.random.uniform(-0.3, 0.3)
    
    linear_speed_2 = np.random.uniform(0.3, 0.7)
    angular_speed_2 = np.random.uniform(-0.3, 0.3)

    rospy.loginfo(f"🔥 Fireball 1: linear={linear_speed_1:.2f}, angular={angular_speed_1:.2f}")
    rospy.loginfo(f"🔥 Fireball 2: linear={linear_speed_2:.2f}, angular={angular_speed_2:.2f}")

    t1 = Twist()
    t1.linear.x = linear_speed_1
    t1.angular.z = angular_speed_1

    t2 = Twist()
    t2.linear.x = linear_speed_2
    t2.angular.z = angular_speed_2

    while not rospy.is_shutdown():
        pub1.publish(t1)
        pub2.publish(t2)
        rate.sleep()

if __name__ == '__main__':
    move_fireballs()
