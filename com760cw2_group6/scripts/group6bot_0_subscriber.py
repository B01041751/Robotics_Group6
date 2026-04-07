#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def force_move():
    rospy.init_node('force_move_test')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    
    move_cmd = Twist()
    move_cmd.linear.x = 1.0  # Full speed forward
    move_cmd.angular.z = 0.5 # Constant turn
    
    rospy.loginfo("Sending FORCE MOVE command... Watch Gazebo!")
    
    while not rospy.is_shutdown():
        pub.publish(move_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        force_move()
    except rospy.ROSInterruptException:
        pass
