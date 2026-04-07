#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist

def oscillate_hazards():
    rospy.init_node('hazard_oscillator')
    
    # Matches the commandTopic in your SDF Planar Move plugins
    pub1 = rospy.Publisher('/hazard_1/cmd_vel', Twist, queue_size=10)
    pub2 = rospy.Publisher('/hazard_2/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(20)
    
    rospy.loginfo("🔥 Fireball Oscillation Started")

    while not rospy.is_shutdown():
        # Using the current ROS time directly handles Gazebo pauses/resets better
        t = rospy.get_time()
        
        # Hazard 1: Moves North-South (Linear X)
        # 0.7 is the speed (amplitude), 1.0 is the frequency (speed of oscillation)
        msg1 = Twist()
        msg1.linear.x = 0.7 * math.sin(1.0 * t) 
        
        # Hazard 2: Moves East-West (Linear Y)
        # Using cos(t) + 1.5 frequency makes it harder for the RL agent to predict
        msg2 = Twist()
        msg2.linear.y = 0.7 * math.cos(1.2 * t)
        
        pub1.publish(msg1)
        pub2.publish(msg2)
        
        try:
            rate.sleep()
        except rospy.ROSTimeMovedBackwardsException:
            # This happens if you reset the Gazebo simulation while the script is running
            pass

    # Safety: Stop hazards when script is closed
    pub1.publish(Twist())
    pub2.publish(Twist())

if __name__ == '__main__':
    try:
        oscillate_hazards()
    except rospy.ROSInterruptException:
        pass
