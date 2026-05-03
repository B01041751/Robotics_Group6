#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from com760cw2_group6.srv import Group6StartBug2, Group6StartBug2Response

GO_TO_GOAL  = "GO_TO_GOAL"
FOLLOW_WALL = "FOLLOW_WALL"
GO_REVERSE  = "GO_REVERSE"
GO_CLOSE    = "GO_CLOSE"

FORWARD_SPEED          = 0.25
WALL_SPEED             = 0.25
GOAL_THRESHOLD         = 0.3
OBSTACLE_DIST          = 0.50
WALL_TARGET            = 0.45
WALL_TOO_CLOSE         = 0.25
WALL_LOST              = 1.5
REVERSE_TRIGGER        = 0.30
WALL_LOST_MAX          = 25
MIN_TRAVEL_BEFORE_LEAVE = 0.3
MLINE_THRESH           = 0.12

KP = 1.5
KD = 1.0


class Bug2:

    def __init__(self):
        rospy.init_node('bug2_controller')

        robot_ns = rospy.get_namespace()

        self.cmd_pub = rospy.Publisher(robot_ns + 'cmd_vel', Twist, queue_size=10)
        rospy.Subscriber(robot_ns + 'scan', LaserScan, self.scan_callback)
        rospy.Subscriber(robot_ns + 'odom', Odometry, self.odom_callback)

        # Relative name — ROS prepends the namespace (/Group6Bot_1/) automatically
        self.service = rospy.Service('start_bug2', Group6StartBug2, self.start_callback)
        rospy.loginfo("Bug2 service ready")

        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        self.goal_x  = 0.0
        self.goal_y  = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.hit_x   = 0.0
        self.hit_y   = 0.0

        self.active = False
        self.mode   = GO_TO_GOAL

        # LiDAR: 720 rays over 180° — index 0=right, 360=front, 719=left
        self.front = 10.0
        self.left  = 10.0

        # M-line coefficients — set when mission starts
        self.mline_a   = 0.0
        self.mline_b   = 0.0
        self.mline_c   = 0.0
        self.mline_len = 0.0

        self.prev_left_err   = 0.0
        self.wall_lost_steps = 0
        self.leave_stamp     = None   # time we last left a wall

        # Auto-start if launch file passed goal params
        if rospy.get_param('~auto_start', False):
            goal_x = float(rospy.get_param('~goal_x', 0.0))
            goal_y = float(rospy.get_param('~goal_y', 0.0))
            rospy.Timer(rospy.Duration(2.0),
                        lambda _: self._auto_start(goal_x, goal_y),
                        oneshot=True)

    def _auto_start(self, goal_x, goal_y):
        rospy.loginfo(f"Auto-starting Bug2 -> goal: ({goal_x:.2f}, {goal_y:.2f})")
        from com760cw2_group6.srv import Group6StartBug2
        rospy.wait_for_service('start_bug2', timeout=10.0)
        svc = rospy.ServiceProxy('start_bug2', Group6StartBug2)
        svc(goal_x, goal_y)

    # ---- SERVICE ----

    def start_callback(self, req):
        self.goal_x  = req.goal_x
        self.goal_y  = req.goal_y
        self.start_x = self.x
        self.start_y = self.y
        self.active  = True
        self.mode    = GO_TO_GOAL
        self.prev_left_err   = 0.0
        self.wall_lost_steps = 0

        # Pre-compute M-line (start→goal) coefficients once per mission
        dx = self.goal_x - self.start_x
        dy = self.goal_y - self.start_y
        self.mline_a   = dy
        self.mline_b   = -dx
        self.mline_c   = dx * self.start_y - dy * self.start_x
        self.mline_len = math.hypot(dx, dy)

        rospy.loginfo(f"Bug2 started -> goal: ({self.goal_x:.2f}, {self.goal_y:.2f})")
        return Group6StartBug2Response(True)

    # ---- CALLBACKS ----

    def odom_callback(self, msg):
        self.x   = msg.pose.pose.position.x
        self.y   = msg.pose.pose.position.y
        o = msg.pose.pose.orientation
        siny = 2.0 * (o.w * o.z + o.x * o.y)
        cosy = 1.0 - 2.0 * (o.y ** 2 + o.z ** 2)
        self.yaw = math.atan2(siny, cosy)

    def scan_callback(self, msg):
        r = msg.ranges
        # Ground-truth indices confirmed from Group6Bot_1_explore.py
        # 720 rays, 180° scan: index 0=right(-90°), 360=front(0°), 719=left(+90°)
        self.front = min([v for v in r[310:410] if math.isfinite(v)] or [10.0])
        self.left  = min([v for v in r[540:720] if math.isfinite(v)] or [10.0])
        self.move()

    # ---- HELPERS ----

    def dist_to_goal(self):
        return math.hypot(self.goal_x - self.x, self.goal_y - self.y)

    def on_m_line(self):
        # Guard: zero-length M-line means start == goal
        if self.mline_len < 1e-6:
            return False
        dist = abs(self.mline_a * self.x + self.mline_b * self.y + self.mline_c) / self.mline_len
        return dist < MLINE_THRESH

    def closer_to_goal(self):
        hit_dist = math.hypot(self.goal_x - self.hit_x, self.goal_y - self.hit_y)
        return self.dist_to_goal() < hit_dist

    def min_travel_met(self):
        return math.hypot(self.x - self.hit_x, self.y - self.hit_y) > MIN_TRAVEL_BEFORE_LEAVE

    @staticmethod
    def clamp(val, lo, hi):
        return max(lo, min(hi, val))

    # ---- CONTROL LOOP ----

    def move(self):
        if not self.active:
            self.cmd_pub.publish(Twist())  # keep broadcasting zero so robot stays stopped
            return

        twist = Twist()
        dist  = self.dist_to_goal()

        if dist < GOAL_THRESHOLD:
            rospy.loginfo("Goal reached!")
            self.active = False
            self.cmd_pub.publish(Twist())
            return

        if   self.mode == GO_TO_GOAL:  self._go_to_goal(twist)
        elif self.mode == FOLLOW_WALL: self._follow_wall(twist)
        elif self.mode == GO_REVERSE:  self._go_reverse(twist)
        elif self.mode == GO_CLOSE:    self._go_close(twist)

        # Simulator requires angular sign inversion on every publish (confirmed in explore.py)
        twist.angular.z = -twist.angular.z
        self.cmd_pub.publish(twist)

    def _go_to_goal(self, twist):
        error = math.atan2(self.goal_y - self.y, self.goal_x - self.x) - self.yaw
        error = math.atan2(math.sin(error), math.cos(error))  # normalize to [-pi, pi]

        twist.linear.x  = FORWARD_SPEED * 0.4 if abs(error) > 0.2 else FORWARD_SPEED
        twist.angular.z = self.clamp(2.0 * error, -1.5, 1.5)

        # Respect post-leave cooldown so we don't re-hit the same wall immediately
        cooling = (self.leave_stamp is not None and
                   (rospy.Time.now() - self.leave_stamp).to_sec() < 2.0)
        if self.front < OBSTACLE_DIST and not cooling:
            rospy.logwarn("Obstacle detected -> FOLLOW_WALL")
            self.hit_x = self.x
            self.hit_y = self.y
            self.prev_left_err   = 0.0
            self.wall_lost_steps = 0
            self.mode = FOLLOW_WALL

    def _follow_wall(self, twist):
        # Priority 1: extremely close front — back up immediately
        if self.front < REVERSE_TRIGGER:
            rospy.logwarn("Front too close -> GO_REVERSE")
            self.mode = GO_REVERSE
            return

        # Priority 2: front blocked — stop and turn right (CW) to clear
        # Left-wall following: concave corner → turn CW (right). Before flip: +1.2 → after flip: -1.2 → right.
        if self.front < OBSTACLE_DIST:
            twist.linear.x  = 0.0
            twist.angular.z = 1.2
        else:
            # Priority 3: PD wall follow on left wall using broad_left_wall [540:720]
            if self.left > WALL_LOST:
                self.wall_lost_steps += 1
                if self.wall_lost_steps >= WALL_LOST_MAX:
                    rospy.logwarn("Wall lost -> GO_CLOSE")
                    self.mode = GO_CLOSE
                    return
                # Convex corner: sweep CCW (left) to stay with wall. Before flip: -0.5 → after flip: +0.5 → left.
                twist.linear.x  = WALL_SPEED
                twist.angular.z = -0.5
            else:
                self.wall_lost_steps = 0
                left_err = WALL_TARGET - self.left
                control  = KP * left_err + KD * (left_err - self.prev_left_err)
                self.prev_left_err = left_err
                twist.linear.x  = WALL_SPEED
                twist.angular.z = self.clamp(control, -1.4, 1.4)

        # Bug 2 leave condition — all four must be true simultaneously
        if (self.min_travel_met()
                and self.on_m_line()
                and self.closer_to_goal()
                and self.front > OBSTACLE_DIST):
            rospy.loginfo("M-line leave condition met -> GO_TO_GOAL")
            self.leave_stamp = rospy.Time.now()
            self.mode = GO_TO_GOAL

    def _go_reverse(self, twist):
        twist.linear.x  = -0.15
        twist.angular.z = 0.0
        if self.front > REVERSE_TRIGGER + 0.1:
            rospy.loginfo("Front clear -> FOLLOW_WALL")
            self.mode = FOLLOW_WALL

    def _go_close(self, twist):
        # Arc CCW (left) to re-acquire lost left wall. Before flip: -0.5 → after flip: +0.5 → left.
        twist.linear.x  = WALL_SPEED
        twist.angular.z = -0.5
        if self.left <= WALL_LOST:
            rospy.loginfo("Wall found -> FOLLOW_WALL")
            self.wall_lost_steps = 0
            self.mode = FOLLOW_WALL


if __name__ == '__main__':
    try:
        Bug2()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
