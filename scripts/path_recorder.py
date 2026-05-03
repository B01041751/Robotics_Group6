#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class PathRecorder:
    def __init__(self):
        rospy.init_node('path_recorder')
        # Change these to match your specific frames/topics
        self.path_pub = rospy.Publisher('/recorded_path', Path, queue_size=10)
        self.odom_sub = rospy.Subscriber('/Group6Bot_0/odom', Odometry, self.odom_callback)
        
        self.path = Path()
        self.path.header.frame_id = "map" # Or "Group6Bot_0/odom"

    def odom_callback(self, data):
        new_pose = PoseStamped()
        new_pose.header = data.header
        new_pose.pose = data.pose.pose
        
        # Append the new position to the path
        self.path.poses.append(new_pose)
        self.path.header.stamp = rospy.Time.now()
        
        # Publish the updated path
        self.path_pub.publish(self.path)

if __name__ == '__main__':
    try:
        PathRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
