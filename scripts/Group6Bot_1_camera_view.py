#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewer:

    def __init__(self):
        rospy.init_node('camera_viewer')
        self.window_title = rospy.get_name()
        self.bridge = CvBridge()
        self.latest_image = None

        rospy.Subscriber('camera/image_raw', Image, self.callback)

    def callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("Error converting image: %s", e)

    def run(self):
        cv2.namedWindow(self.window_title, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_title, 640, 480)

        # Position Bot0 top-left, Bot1 top-right on a 1920x1080 screen
        if 'Bot0' in self.window_title:
            cv2.moveWindow(self.window_title, 0, 0)
        else:
            cv2.moveWindow(self.window_title, 1280, 0)

        window_shown = False
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            if self.latest_image is not None:
                cv2.imshow(self.window_title, self.latest_image)
                window_shown = True

            cv2.waitKey(1)

            # WND_PROP_AUTOSIZE returns -1 only when the window is fully destroyed.
            # Unlike WND_PROP_VISIBLE, it does not return -1 on a freshly created window.
            if window_shown and cv2.getWindowProperty(self.window_title, cv2.WND_PROP_AUTOSIZE) < 0:
                rospy.signal_shutdown("Camera window closed by user")
                break

            rate.sleep()

        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        viewer = CameraViewer()
        viewer.run()
    except rospy.ROSInterruptException:
        pass
