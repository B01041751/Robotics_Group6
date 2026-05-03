#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist

def oscillate_hazards(): # This part of the code initialises a ROS node that is responsible for driving the hazard motion. 
# This node also runs independently of the RL agent and the Gazebo plugins. 
    rospy.init_node('hazard_oscillator')
    # The publishers match the command topics defined in the hazard_world SDF file. 
    # The Planar Move plugins for both hazard_1 and hazard_2. 
     
    # Matching the commandTopic's in SDF using Planar Move plugins
    pub1 = rospy.Publisher('/hazard_1/cmd_vel', Twist, queue_size=10)
    pub2 = rospy.Publisher('/hazard_2/cmd_vel', Twist, queue_size=10)
    #  20 Hz has been set for the update rate which is smooth enough for continous oscillation without overloading the Gazebo or the ROS.
    rate = rospy.Rate(20)
    
    rospy.loginfo("Fireball Oscillation Started")
    # The main control loop will run until the ROS shutdown has been initiated. 

    while not rospy.is_shutdown():
        # Using the ROS time instead of the python time will ensure:
        # - That pauses in the Gazebo freeze the oscillation.
        # - It will reset the rewind timing correctly.
        # - It allows the RL episodes to remain synchronised with hazard motion. 
        t = rospy.get_time()
        
        # The Hazard 1 part, this defines the North-South motion (Linear X).
        # The Twist() is defaults are 0 for all of the fields. 
        # Hazard 1: clockwise circle — vx = -A*sin(w*t), vy = A*cos(w*t)
        # Circular motion avoids getting stuck against a single obstacle the way
        # straight-line oscillation does.
        msg1 = Twist()
        msg1.linear.x = -0.7 * math.sin(0.8 * t)
        msg1.linear.y =  0.7 * math.cos(0.8 * t)

        # Hazard 2: counter-clockwise, slightly different speed for varied behaviour
        msg2 = Twist()
        msg2.linear.x =  0.6 * math.sin(1.1 * t)
        msg2.linear.y = -0.6 * math.cos(1.1 * t)
        
        pub1.publish(msg1)
        pub2.publish(msg2)
        
        try:
        # The sleep is to maintain the 20 Hz loop rate. 
            rate.sleep()
        except rospy.ROSTimeMovedBackwardsException:
            # The Gazebo resets causing the ROS time to jump backwards without having this catch, the node would crash.
            # It then would ignore the exception which then allows the oscillation to resume cleanly. 
            pass

    # Safetly shutting down.
    # If the node exits normally - control c or shutdown etc, it will send zero-velocity commands to the stop hazards immediately. 
    pub1.publish(Twist())
    pub2.publish(Twist())

if __name__ == '__main__':
    try:
        oscillate_hazards()
    except rospy.ROSInterruptException:
    # This is the standard ROS shutdown handling that is used. 
        pass
