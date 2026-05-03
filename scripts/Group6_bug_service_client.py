#!/usr/bin/env python3
# rospy is the ROS Python library used for nodes and services
import rospy
# Import the custom service used to control Bug behaviour
from com760cw2_group6.srv import SetBugStatus

# Function used to call the Bug control service
def call_service():
# Wait until the service 'set_bug_status' is available
    rospy.wait_for_service('set_bug_status')

    try:
    # Create a service proxy to communicate with the service server
        service = rospy.ServiceProxy('set_bug_status', SetBugStatus)

	
	# Call the service with desired parameters
        # True  -> enable bug behaviour
        # 0.3   -> robot forward speed
        # "left"-> turning direction

        response = service(True, 0.3, "left")

	# Print the response message received from the service server
        print("Response:", response.message)

    except rospy.ServiceException as e:
    	# Print error message if service call fails
        print("Service call failed:", e)

# Main entry point of the script
if __name__ == '__main__':
    # Initialize the ROS node with a unique name
    rospy.init_node('bug_service_client')
    # Call the service once
    call_service()
