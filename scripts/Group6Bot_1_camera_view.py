#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewer:

    def __init__(self):
        rospy.init_node('camera_viewer')

        self.bridge = CvBridge()

        rospy.Subscriber('camera/image_raw', Image, self.callback)

    def callback(self, msg):
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Show image
            cv2.imshow("Robot Camera", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr("Error: %s", e)

if __name__ == '__main__':
    try:
        CameraViewer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
