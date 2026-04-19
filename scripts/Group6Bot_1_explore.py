#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import tf

class Bug2Navigator:
    """
    Bug2 Algorithm Implementation for autonomous navigation.
    Combines go-to-goal behavior with wall-following for obstacle avoidance.
    """

    def __init__(self):
        rospy.init_node('bug2_navigator')

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('scan', LaserScan, self.scan_callback)
        rospy.Subscriber('odom', Odometry, self.odom_callback)

        # Goal position
        self.goal_x = 5.0
        self.goal_y = 5.0
        
        # Current pose
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.yaw = 0.0
        
        # Bug2 state
        self.state = "GO_TO_GOAL"  # GO_TO_GOAL or FOLLOW_WALL
        self.hit_point_x = 0.0
        self.hit_point_y = 0.0
        self.wall_side = None  # "left" or "right"
        
        # Laser data
        self.ranges = []
        self.front_dist = 1.0
        self.left_dist = 1.0
        self.right_dist = 1.0

        rospy.loginfo("🐛 Bug2 Navigator initialized")

    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.yaw = tf.transformations.euler_from_quaternion(q)[2]

    def scan_callback(self, msg):
        """Process laser scan and update distances"""
        ranges = msg.ranges
        ranges = [r if not math.isinf(r) and not math.isnan(r) else 10.0 for r in ranges]
        
        n = len(ranges)
        
        # Divide laser into 3 regions
        self.front_dist = min(ranges[max(0, n//2 - 40):min(n, n//2 + 40)])
        self.left_dist = min(ranges[max(0, n//2 + 40):min(n, n//2 + 120)])
        self.right_dist = min(ranges[max(0, n//2 - 120):min(n, n//2 - 40)])
        
        self.ranges = ranges
        self.navigate()

    def distance_to_goal(self):
        """Calculate distance to goal"""
        dx = self.goal_x - self.pose_x
        dy = self.goal_y - self.pose_y
        return math.sqrt(dx**2 + dy**2)

    def angle_to_goal(self):
        """Calculate angle to goal"""
        dx = self.goal_x - self.pose_x
        dy = self.goal_y - self.pose_y
        return math.atan2(dy, dx)

    def angle_diff(self, target_angle):
        """Calculate shortest angle difference"""
        diff = target_angle - self.yaw
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def navigate(self):
        """Main Bug2 navigation logic"""
        twist = Twist()
        dist_to_goal = self.distance_to_goal()
        angle_to_goal = self.angle_to_goal()
        angle_diff = self.angle_diff(angle_to_goal)

        # Goal reached
        if dist_to_goal < 0.3:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            rospy.loginfo("✅ Goal reached!")
            self.pub.publish(twist)
            return

        # State machine
        if self.state == "GO_TO_GOAL":
            # Try to go straight to goal
            if self.front_dist > 0.6:
                # Path is clear, move towards goal
                twist.linear.x = 0.25
                twist.angular.z = np.clip(angle_diff, -1.0, 1.0)
                rospy.loginfo(f"🎯 Going to goal (dist={dist_to_goal:.2f})")
            else:
                # Obstacle detected, switch to wall following
                self.state = "FOLLOW_WALL"
                self.hit_point_x = self.pose_x
                self.hit_point_y = self.pose_y
                self.wall_side = "left" if self.left_dist < self.right_dist else "right"
                rospy.loginfo(f"🚧 Hit obstacle! Following wall on {self.wall_side}")

        elif self.state == "FOLLOW_WALL":
            # Follow wall boundary
            if self.wall_side == "left":
                # Wall on left: keep it close and move forward
                if self.left_dist < 0.4:
                    twist.linear.x = 0.2
                    twist.angular.z = 0.3  # Turn away from wall
                elif self.left_dist > 0.8:
                    twist.linear.x = 0.2
                    twist.angular.z = -0.3  # Turn towards wall
                else:
                    twist.linear.x = 0.2
                    twist.angular.z = 0.0  # Maintain distance

            else:  # wall_side == "right"
                # Wall on right: keep it close and move forward
                if self.right_dist < 0.4:
                    twist.linear.x = 0.2
                    twist.angular.z = -0.3  # Turn away from wall
                elif self.right_dist > 0.8:
                    twist.linear.x = 0.2
                    twist.angular.z = 0.3  # Turn towards wall
                else:
                    twist.linear.x = 0.2
                    twist.angular.z = 0.0  # Maintain distance

            # Check if we can return to goal
            m_line_dist = self.point_to_line_distance()
            current_goal_dist = dist_to_goal
            hit_goal_dist = math.sqrt((self.goal_x - self.hit_point_x)**2 + 
                                     (self.goal_y - self.hit_point_y)**2)
            
            # Leave wall when closer to goal than at hit point
            if current_goal_dist < hit_goal_dist and self.front_dist > 0.6:
                self.state = "GO_TO_GOAL"
                rospy.loginfo(f"↩️ Leaving wall, back to goal-seeking (dist={current_goal_dist:.2f})")

        # Stuck condition
        if self.front_dist < 0.3 and self.left_dist < 0.3 and self.right_dist < 0.3:
            rospy.logwarn("⚠️ Stuck! Spinning to escape...")
            twist.linear.x = 0.05
            twist.angular.z = 1.5

        self.pub.publish(twist)

    def point_to_line_distance(self):
        """Distance from robot to M-line (goal line)"""
        # M-line is the line from start (0,0) to goal
        x0, y0 = self.pose_x, self.pose_y
        x1, y1 = 0, 0
        x2, y2 = self.goal_x, self.goal_y
        
        num = abs((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1)
        den = math.sqrt((y2-y1)**2 + (x2-x1)**2)
        
        return num / den if den != 0 else 0


if __name__ == '__main__':
    try:
        navigator = Bug2Navigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

