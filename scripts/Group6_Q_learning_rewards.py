#!/usr/bin/env python3
# rospy is the main ROS Python library for nodes and communication
# Import the custom QLearning message used for reinforcement learning
import rospy
from com760cw2_group6.msg import QLearning

# This function publishes Q-learning data continuously
def publisher():
    # Create a publisher that publishes QLearning messages on 'q_learning' topic
    pub = rospy.Publisher('q_learning', QLearning, queue_size=10)
    # Initialize the ROS node with a unique name
    rospy.init_node('q_learning_publisher', anonymous=True)

    # Set the loop rate to 10 Hz
    rate = rospy.Rate(10)

    # Keep publishing until ROS shuts down
    while not rospy.is_shutdown():
    	# Create a QLearning message
        msg = QLearning()
        
        # Set the state number (example value)
        msg.state = 1
        
        # Set the action taken by the robot (example value)
        msg.action = 2
        
        # Set the reward received after taking the action
        msg.reward = 0.5
        
        # Indicates whether the episode has finished
        msg.done = False

	# Publish the message on the q_learning topic
        pub.publish(msg)
        
        # Log a message to indicate publishing
        rospy.loginfo("Published QLearning message")

	# Sleep to maintain the publishing rate
        rate.sleep()

# Entry point of the script
if __name__ == '__main__':
    # Call the publisher function to start publishing messages
    publisher()
