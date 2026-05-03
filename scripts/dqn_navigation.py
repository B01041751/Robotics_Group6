#!/usr/bin/env python3
import math
import rospy
import pygame
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


class HazardNavigator:
    def __init__(self):
        rospy.init_node('hazard_path_follower')

        # Publishers / subscribers
        self.cmd_pub = rospy.Publisher('/Group6Bot_0/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/Group6Bot_0/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/Group6Bot_0/scan', LaserScan, self.scan_cb)

        # State
        self.pose = None
        self.yaw = 0.0
        self.scan = []
        self.stage = 0

        # Path waypoints (example polygon ending at gas cloud)
        # Adjust these if your start pose differs
        self.waypoints = [
            (0.0, 5.5),   # approach tunnel
            (0.0, 8.5),   # inside tunnel
            (1.5, -3.5),  # exit region
            (2.0, -6.0)   # gas cloud centre
        ]

        rospy.on_shutdown(self.stop)

        # Audio
        pygame.mixer.init()
        self.alarm = None
        try:
            pygame.mixer.music.load('/home/ubuntu/com760cw2_group6/src/com760cw2_group6/sound/beto.wav')
            pygame.mixer.music.play(-1)
            self.alarm = pygame.mixer.Sound('/home/ubuntu/com760cw2_group6/src/com760cw2_group6/sound/alarm.wav')
        except Exception as e:
            rospy.logwarn("Audio init failed: %s", str(e))

    # ----------------- Callbacks -----------------

    def odom_cb(self, msg):
        self.pose = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def scan_cb(self, msg):
        self.scan = list(msg.ranges)

    # ----------------- Helpers -----------------

    def stop(self):
        self.cmd_pub.publish(Twist())

    def sectors(self):
        """Return (left, front, right) distances from LiDAR."""
        if not self.scan:
            return 5.0, 5.0, 5.0

        n = len(self.scan)
        width = 10

        def clean(vals):
            vals = [v for v in vals if not math.isinf(v) and not math.isnan(v)]
            return min(vals) if vals else 5.0

        # Assuming scan[0] is forward, angles increase CCW
        front_idx = 0
        left_idx = int(n * 0.25)   # ~90°
        right_idx = int(n * 0.75)  # ~270°

        # Wrap-around for front
        if front_idx - width < 0:
            front_vals = self.scan[-width:] + self.scan[0:front_idx + width]
        else:
            front_vals = self.scan[front_idx - width:front_idx + width]

        left_vals = self.scan[left_idx - width:left_idx + width]
        right_vals = self.scan[right_idx - width:right_idx + width]

        return clean(left_vals), clean(front_vals), clean(right_vals)

    # ----------------- Navigation -----------------

    def goto_waypoint(self, gx, gy):
        dx = gx - self.pose.x
        dy = gy - self.pose.y
        dist = math.hypot(dx, dy)

        target = math.atan2(dy, dx)
        err = math.atan2(math.sin(target - self.yaw), math.cos(target - self.yaw))

        left, front, right = self.sectors()
        cmd = Twist()

        # Obstacle directly ahead
        if front < 0.65:
            cmd.linear.x = -0.05
            cmd.angular.z = 0.9 if left > right else -0.9
            self.cmd_pub.publish(cmd)
            return dist

        # Tunnel mode: both walls close → centre in corridor
        if left < 0.6 and right < 0.6:
            cmd.linear.x = 0.10
            cmd.angular.z = (right - left) * 1.2
            self.cmd_pub.publish(cmd)
            return dist

        # Normal goal-seeking
        cmd.angular.z = max(-1.0, min(1.0, 1.8 * err))
        cmd.linear.x = max(0.08, 0.22 * (1 - abs(err)))

        # Wall repulsion
        if left < 0.55:
            cmd.angular.z -= 0.8 * (0.55 - left)
        if right < 0.55:
            cmd.angular.z += 0.8 * (0.55 - right)

        self.cmd_pub.publish(cmd)
        return dist

    # ----------------- Main loop -----------------

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.pose is None:
                rate.sleep()
                continue

            if self.stage < len(self.waypoints):
                gx, gy = self.waypoints[self.stage]
                dist = self.goto_waypoint(gx, gy)

                if dist < 0.5:
                    self.stage += 1
                    rospy.loginfo("Reached waypoint %d", self.stage)
            else:
                # Only once the final waypoint (gas cloud) is reached
                self.stop()
                if self.alarm:
                    self.alarm.play()
                rospy.loginfo("Gas cloud reached. Alarm activated.")
                rospy.sleep(5)
                break

            rate.sleep()


if __name__ == '__main__':
    HazardNavigator().run()

