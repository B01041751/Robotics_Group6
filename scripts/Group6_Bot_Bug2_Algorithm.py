#!/usr/bin/env python3

import math
import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from com760cw2_group6.srv import (
    Group6StartBug2,
    Group6StartBug2Response
)
from com760cw2_group6.msg import BotAlert

GO_TO_GOAL = "GO_TO_GOAL"
FOLLOW_WALL = "FOLLOW_WALL"
GO_REVERSE = "GO_REVERSE"
GO_CLOSE = "GO_CLOSE"

FORWARD_SPEED = 0.25
WALL_SPEED = 0.25

GOAL_THRESHOLD = 0.3

OBSTACLE_DIST = 0.50
WALL_TARGET = 0.45
WALL_TOO_CLOSE = 0.25
WALL_LOST = 1.5

REVERSE_TRIGGER = 0.30

WALL_LOST_MAX = 25

MIN_TRAVEL_BEFORE_LEAVE = 0.3

# Slightly relaxed threshold helps Bug2 leave wall correctly
MLINE_THRESH = 0.40

KP = 1.5
KD = 0.4


class Bug2:

    def __init__(self):

        rospy.init_node('bug2_controller')

        robot_ns = rospy.get_namespace()

        self.cmd_pub = rospy.Publisher(
            robot_ns + 'cmd_vel',
            Twist,
            queue_size=10
        )

        rospy.Subscriber(
            robot_ns + 'scan',
            LaserScan,
            self.scan_callback
        )

        rospy.Subscriber(
            robot_ns + 'odom',
            Odometry,
            self.odom_callback
        )

        self.service = rospy.Service(
            'start_bug2',
            Group6StartBug2,
            self.start_callback
        )

        self._alert_pub = rospy.Publisher('alert', BotAlert, queue_size=10)
        rospy.Subscriber('/Group6Bot_0/alert', BotAlert, self._bot0_alert_cb)

        rospy.loginfo("Bug2 service ready")

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.goal_x = 0.0
        self.goal_y = 0.0

        self.start_x = 0.0
        self.start_y = 0.0

        self.hit_x = 0.0
        self.hit_y = 0.0

        self.active = False
        self.mode = GO_TO_GOAL

        # LiDAR
        self.front = 10.0
        self.left = 10.0

        # M-line
        self.mline_a = 0.0
        self.mline_b = 0.0
        self.mline_c = 0.0
        self.mline_len = 0.0

        self.prev_left_err = 0.0
        self.wall_lost_steps = 0

        self.leave_stamp = None

        # -------------------------------------------------
        # DEBUG / STUCK DETECTION
        # -------------------------------------------------

        self.prev_goal_dist = 9999.0
        self.last_progress_time = rospy.Time.now()

        self.last_position = (0.0, 0.0)
        self.stuck_counter = 0

        # -------------------------------------------------
        # AUTO START
        # -------------------------------------------------

        if rospy.get_param('~auto_start', False):
            goal_x = float(
                rospy.get_param('~goal_x', 0.0)
            )

            goal_y = float(
                rospy.get_param('~goal_y', 0.0)
            )

            rospy.Timer(
                rospy.Duration(2.0),
                lambda _: self._auto_start(goal_x, goal_y),
                oneshot=True
            )

    # -------------------------------------------------
    # AUTO START
    # -------------------------------------------------

    def _auto_start(self, goal_x, goal_y):

        rospy.loginfo(
            f"Auto-starting Bug2 -> "
            f"goal: ({goal_x:.2f}, {goal_y:.2f})"
        )

        rospy.wait_for_service(
            'start_bug2',
            timeout=10.0
        )

        svc = rospy.ServiceProxy(
            'start_bug2',
            Group6StartBug2
        )

        svc(goal_x, goal_y)

    def _publish_alert(self, alert_type, x, y):
        msg = BotAlert()
        msg.bot_id     = 'bot1'
        msg.alert_type = alert_type
        msg.x          = float(x)
        msg.y          = float(y)
        self._alert_pub.publish(msg)

    _ALERT_NAMES = {BotAlert.GAS: 'GAS', BotAlert.FIREBALL: 'FIREBALL', BotAlert.OBSTACLE: 'OBSTACLE'}

    def _bot0_alert_cb(self, msg):
        label = self._ALERT_NAMES.get(msg.alert_type, str(msg.alert_type))
        rospy.loginfo(f"[Bot1] [Custom Message] from {msg.bot_id}: {label} at ({msg.x:.2f}, {msg.y:.2f})")

    # -------------------------------------------------
    # SERVICE
    # -------------------------------------------------

    def start_callback(self, req):

        self.goal_x = req.goal_x
        self.goal_y = req.goal_y

        self.start_x = self.x
        self.start_y = self.y

        self.active = True
        self.mode = GO_TO_GOAL

        self.prev_left_err = 0.0
        self.wall_lost_steps = 0

        self.prev_goal_dist = 9999.0
        self.last_progress_time = rospy.Time.now()

        dx = self.goal_x - self.start_x
        dy = self.goal_y - self.start_y

        self.mline_a = dy
        self.mline_b = -dx
        self.mline_c = dx * self.start_y - dy * self.start_x

        self.mline_len = math.hypot(dx, dy)

        rospy.loginfo(
            f"Bug2 started -> "
            f"goal: ({self.goal_x:.2f}, {self.goal_y:.2f})"
        )

        return Group6StartBug2Response(True)

    # -------------------------------------------------
    # CALLBACKS
    # -------------------------------------------------

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        o = msg.pose.pose.orientation

        siny = 2.0 * (o.w * o.z + o.x * o.y)
        cosy = 1.0 - 2.0 * (o.y ** 2 + o.z ** 2)

        self.yaw = math.atan2(siny, cosy)

    def scan_callback(self, msg):

        r = msg.ranges

        self.front = min(
            [v for v in r[310:410] if math.isfinite(v)] or [10.0]
        )

        self.left = min(
            [v for v in r[540:720] if math.isfinite(v)] or [10.0]
        )

        self.move()

    # -------------------------------------------------
    # HELPERS
    # -------------------------------------------------

    def dist_to_goal(self):

        return math.hypot(
            self.goal_x - self.x,
            self.goal_y - self.y
        )

    def on_m_line(self):

        if self.mline_len < 1e-6:
            return False

        dist = abs(
            self.mline_a * self.x +
            self.mline_b * self.y +
            self.mline_c
        ) / self.mline_len

        return dist < MLINE_THRESH

    def closer_to_goal(self):

        hit_dist = math.hypot(
            self.goal_x - self.hit_x,
            self.goal_y - self.hit_y
        )

        return self.dist_to_goal() < hit_dist

    def min_travel_met(self):

        return math.hypot(
            self.x - self.hit_x,
            self.y - self.hit_y
        ) > MIN_TRAVEL_BEFORE_LEAVE

    @staticmethod
    def clamp(val, lo, hi):

        return max(lo, min(hi, val))

    # -------------------------------------------------
    # CONTROL LOOP
    # -------------------------------------------------

    def move(self):

        if not self.active:
            self.cmd_pub.publish(Twist())
            return

        twist = Twist()

        dist = self.dist_to_goal()

        # -------------------------------------------------
        # GOAL REACHED
        # -------------------------------------------------

        if dist < GOAL_THRESHOLD:
            rospy.loginfo("SUCCESS: Goal reached!")
            self._publish_alert(BotAlert.GAS, self.goal_x, self.goal_y)

            self.active = False
            self.cmd_pub.publish(Twist())

            return

        # -------------------------------------------------
        # MOVEMENT CHECK
        # -------------------------------------------------

        moved = math.hypot(
            self.x - self.last_position[0],
            self.y - self.last_position[1]
        )

        if moved > 0.03:

            self.last_position = (self.x, self.y)
            self.stuck_counter = 0

        else:

            self.stuck_counter += 1

        # -------------------------------------------------
        # PROGRESS CHECK
        # -------------------------------------------------

        if dist < self.prev_goal_dist - 0.02:
            self.prev_goal_dist = dist
            self.last_progress_time = rospy.Time.now()

        no_progress = (
                rospy.Time.now() - self.last_progress_time
        ).to_sec()

        # -------------------------------------------------
        # STUCK DETECTION
        # -------------------------------------------------

        if self.stuck_counter > 80:

            rospy.logwarn(
                f"Robot stuck detected | "
                f"mode={self.mode} | "
                f"front={self.front:.2f} | "
                f"left={self.left:.2f}"
            )

            # Reset counter
            self.stuck_counter = 0

            # Force attempt toward goal again
            if self.front > 0.3:

                rospy.logwarn(
                    "Recovery: switching to GO_TO_GOAL"
                )

                self.mode = GO_TO_GOAL
                self.stuck_counter = 0

            else:

                rospy.logwarn(
                    "Recovery: obstacle still ahead"
                )

                self.mode = FOLLOW_WALL

        rospy.loginfo_throttle(
            2.0,
            f"[DEBUG] "
            f"Robot=({self.x:.2f},{self.y:.2f}) "
            f"Goal=({self.goal_x:.2f},{self.goal_y:.2f}) "
            f"Dist={dist:.2f} "
            f"Mode={self.mode}"
        )

        if no_progress > 30.0:
            # Force attempt toward goal again
            rospy.logwarn(
                    f"self.front: {self.front}"
                )
            if self.front > 0.7:

                rospy.logwarn(
                    "Recovery: switching to GO_TO_GOAL"
                )

                self.mode = GO_TO_GOAL

            else:

                rospy.logwarn(
                    "Recovery: obstacle still ahead"
                )

                self.mode = FOLLOW_WALL
            rospy.logerr(
                "FAILED: Robot cannot reach goal.\n"
                f"Current mode: {self.mode}\n"
                f"Distance to goal: {dist:.2f}\n"
                f"Front obstacle distance: {self.front:.2f}\n"
                f"Left wall distance: {self.left:.2f}\n"
                "Reason: No progress toward goal for 30 seconds."
            )

            self.active = False
            self.cmd_pub.publish(Twist())

            return

        # -------------------------------------------------
        # NORMAL BUG2 LOGIC
        # -------------------------------------------------

        if self.mode == GO_TO_GOAL:

            self._go_to_goal(twist)

        elif self.mode == FOLLOW_WALL:

            self._follow_wall(twist)

        elif self.mode == GO_REVERSE:

            self._go_reverse(twist)

        elif self.mode == GO_CLOSE:

            self._go_close(twist)

        # Simulator sign inversion
        twist.angular.z = -twist.angular.z

        self.cmd_pub.publish(twist)

    # -------------------------------------------------
    # GO TO GOAL
    # -------------------------------------------------

    def _go_to_goal(self, twist):

        error = math.atan2(
            self.goal_y - self.y,
            self.goal_x - self.x
        ) - self.yaw

        error = math.atan2(
            math.sin(error),
            math.cos(error)
        )

        twist.linear.x = (
            FORWARD_SPEED * 0.4
            if abs(error) > 0.2
            else FORWARD_SPEED
        )

        twist.angular.z = self.clamp(
            2.0 * error,
            -1.5,
            1.5
        )

        rospy.loginfo_throttle(
            2.0,
            f"[GO_TO_GOAL] "
            f"dist={self.dist_to_goal():.2f} "
            f"front={self.front:.2f} "
            f"heading_error={error:.2f}"
        )

        # Cooldown
        cooling = (
                self.leave_stamp is not None and
                (rospy.Time.now() - self.leave_stamp).to_sec() < 4.0
        )

        if self.front < OBSTACLE_DIST and not cooling:
            rospy.logwarn(
                "Obstacle detected -> FOLLOW_WALL"
            )

            self.hit_x = self.x
            self.hit_y = self.y

            self.prev_left_err = 0.0
            self.wall_lost_steps = 0

            self.mode = FOLLOW_WALL
            self._publish_alert(BotAlert.OBSTACLE, self.x, self.y)

    # -------------------------------------------------
    # FOLLOW WALL
    # -------------------------------------------------

    def _follow_wall(self, twist):

        # Emergency reverse
        if self.front < REVERSE_TRIGGER:
            rospy.logwarn(
                "Front too close -> GO_REVERSE"
            )

            self.mode = GO_REVERSE

            return

        # Front blocked
        if self.front < OBSTACLE_DIST:

            twist.linear.x = 0.0
            twist.angular.z = 1.2

        else:

            # Wall lost
            if self.left > WALL_LOST:

                self.wall_lost_steps += 1

                if self.wall_lost_steps >= WALL_LOST_MAX:
                    rospy.logwarn(
                        "Wall lost -> GO_CLOSE"
                    )

                    self.mode = GO_CLOSE

                    return

                twist.linear.x = WALL_SPEED
                twist.angular.z = -0.5

            else:

                self.wall_lost_steps = 0

                left_err = WALL_TARGET - self.left

                control = (
                        KP * left_err +
                        KD * (
                                left_err -
                                self.prev_left_err
                        )
                )

                self.prev_left_err = left_err

                twist.linear.x = WALL_SPEED

                twist.angular.z = self.clamp(
                    control,
                    -1.4,
                    1.4
                )

        rospy.loginfo_throttle(
            2.0,
            f"[FOLLOW_WALL] "
            f"front={self.front:.2f} "
            f"left={self.left:.2f} "
            f"on_mline={self.on_m_line()} "
            f"closer={self.closer_to_goal()} "
            f"travel={self.min_travel_met()}"
        )

        # Bug2 leave condition
        if (
                self.min_travel_met()
                and self.on_m_line()
                and self.closer_to_goal()
                and self.front > 0.40
        ):
            rospy.loginfo(
                "M-line leave condition met "
                "-> GO_TO_GOAL"
            )

            self.leave_stamp = rospy.Time.now()

            self.mode = GO_TO_GOAL

    # -------------------------------------------------
    # REVERSE
    # -------------------------------------------------

    def _go_reverse(self, twist):

        twist.linear.x = -0.15
        twist.angular.z = 0.0

        if self.front > REVERSE_TRIGGER + 0.1:
            rospy.loginfo(
                "Front clear -> FOLLOW_WALL"
            )

            self.mode = FOLLOW_WALL

    # -------------------------------------------------
    # GO CLOSE
    # -------------------------------------------------

    def _go_close(self, twist):

        twist.linear.x = WALL_SPEED
        twist.angular.z = -0.5

        if self.left <= WALL_LOST:
            rospy.loginfo(
                "Wall found -> FOLLOW_WALL"
            )

            self.wall_lost_steps = 0

            self.mode = FOLLOW_WALL

            return


if __name__ == '__main__':

    try:

        Bug2()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
