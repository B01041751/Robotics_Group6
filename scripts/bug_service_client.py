#!/usr/bin/env python3

import rospy
from com760cw2_group6.srv import SetBugStatus

def call_service():
    rospy.wait_for_service('set_bug_status')

    try:
        service = rospy.ServiceProxy('set_bug_status', SetBugStatus)

        response = service(True, 0.3, "left")

        print("Response:", response.message)

    except rospy.ServiceException as e:
        print("Service call failed:", e)


if __name__ == '__main__':
    rospy.init_node('bug_service_client')
    call_service()
