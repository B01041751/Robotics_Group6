#!/usr/bin/env python3
# rospy is the main ROS Python library used to create nodes and services
# Import the custom service used to control Bug behaviour
# Twist message is used to send velocity commands to the robot
import rospy
from com760cw2_group6.srv import SetBugStatus, SetBugStatusResponse
from geometry_msgs.msg import Twist

# BugController class controls robot movement based on a service request
class BugController:

    def __init__(self):
    	 # Initialize the ROS node with a unique name
        rospy.init_node('bug_service_server')

	# Publisher to send velocity commands to the robot
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

	# Variables to store bug behaviour state
	# Whether bug behaviour is enabled
        self.active = False
        # Linear speed of the robot
        self.speed = 0.2
        # Direction to turn (left or right)
        self.direction = "left"
		
	# Create a ROS service called 'set_bug_status'
        # The callback method is called whenever the service is used

        rospy.Service('set_bug_status', SetBugStatus, self.callback)

    def callback(self, req):
    	# Update internal state using values from the service request
        self.active = req.flag
        self.speed = req.speed
        self.direction = req.direction

	# Print service values to the ROS terminal for debugging
        rospy.loginfo(f"Bug behaviour: {self.active}, speed={self.speed}, dir={self.direction}")

	# Send response back to the service client
        return SetBugStatusResponse("Service updated successfully")

	#  Main loop to control robot movement
    def run(self):
    	# Set loop rate to 10 Hz
        rate = rospy.Rate(10)

	# Keep running until ROS is shut down
        while not rospy.is_shutdown():
        # Create a Twist message
            twist = Twist()

	# Only move the robot if bug behaviour is active
            if self.active:
                twist.linear.x = self.speed

		# Turn left or right based on service input
                if self.direction == "left":
                    twist.angular.z = 0.5
                elif self.direction == "right":
                    twist.angular.z = -0.5

	# Publish velocity command
            self.pub.publish(twist)
            # Sleep to maintain loop rate
            rate.sleep()

# Main entry point of the program
if __name__ == '__main__':
# Create BugController object
    node = BugController()
    # Start the control loop
    node.run()
