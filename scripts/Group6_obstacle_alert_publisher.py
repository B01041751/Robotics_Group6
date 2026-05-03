#!/usr/bin/env python3
# rospy is the main ROS Python library for nodes and communication
# LaserScan message is used to receive distance data from the LiDAR sensor
# Point message is used to represent a position in 3D space (x, y, z)
# Math library is used for mathematical calculations such as distance or angle computation
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import math

class ObstacleAlertPublisher:

    def __init__(self):
        # initialise ROS node
        rospy.init_node('obstacle_alert_publisher')

        # publisher to send obstacle position (x, y) to other robot
        self.pub = rospy.Publisher('/obstacle_alert', Point, queue_size=10)

        # subscribe to LiDAR scan data
        rospy.Subscriber('scan', LaserScan, self.scan_callback)

    def scan_callback(self, msg):

        # get all distance readings from LiDAR
        ranges = msg.ranges
        n = len(ranges)

        # only check front region of robot (approx middle 60 readings)
        start = n//2 - 30
        end   = n//2 + 30

        # initialise minimum distance as very large value
        min_dist = float('inf')
        min_index = 0

        # loop through front region to find closest obstacle
        for i in range(start, end):
            r = ranges[i]

            # ignore invalid values (inf or NaN)
            if math.isinf(r) or math.isnan(r):
                continue

            # find smallest distance (closest object)
            if r < min_dist:
                min_dist = r
                min_index = i

        # if obstacle is within threshold distance
        if min_dist < 0.8:

            # calculate angle of that obstacle
            angle = msg.angle_min + min_index * msg.angle_increment

            # convert from polar (r, angle) to Cartesian (x, y)
            x = min_dist * math.cos(angle)
            y = min_dist * math.sin(angle)

            # create message to send
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0

            # print for debugging
            rospy.logwarn(f"Robot1 detected obstacle at (x={x:.2f}, y={y:.2f})")

            # publish obstacle position
            self.pub.publish(point)


if __name__ == '__main__':
    try:
        # create object of class
        ObstacleAlertPublisher()

        # keep node running
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
