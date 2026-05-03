#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ── Tuning constants ──────────────────────────────────────────────────────────
GOAL_THRESHOLD  = 0.3
OBSTACLE_DIST   = 0.55
WALL_DIST       = 0.45
FAR_WALL_LIMIT  = 1.5
CLOSE_FRONT_LIM = 0.5

MIN_HIT_DIST    = 0.5
MLINE_THRESH    = 0.1

FAR_COUNT_MAX   = 20
CLOSE_COUNT_MAX = 20

ALIGN_THRESH    = 0.05
LINEAR_SPD      = 0.4
ANGULAR_SPD     = 1.4

# ── States ────────────────────────────────────────────────────────────────────
ALIGN_TO_GOAL  = 'align_to_goal'
MOVE_TO_GOAL   = 'move_to_goal'
WALL_FOLLOWING = 'wall_following'
GO_REVERSE     = 'go_reverse'
GO_CLOSE       = 'go_close'
TURN_CORNER    = 'turn_corner'


def _yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _norm_angle(a):
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


class Navigator:

    def __init__(self):
        rospy.init_node('explorer')

        self.goal_x = float(rospy.get_param('~gas_x', 0.0))
        self.goal_y = float(rospy.get_param('~gas_y', 0.0))

        self.x = self.y = self.yaw = 0.0
        self.pose_ready = False

        self.start_x = None
        self.start_y = None
        self.mline_ready = False

        self.front = 10.0
        self.left  = 10.0

        self.state = ALIGN_TO_GOAL

        self.hit_point             = None
        self.far_from_wall_counter = 0
        self.close_counter         = 0

        self.prev_left_err = 0.0

        # Corner handling
        self.turn_phase = None
        self.turn_start_time = None
        self.last_corner_finish_time = None

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('odom', Odometry, self.odom_cb)
        rospy.Subscriber('scan', LaserScan, self.scan_cb)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def odom_cb(self, msg):
        self.x   = msg.pose.pose.position.x
        self.y   = msg.pose.pose.position.y
        self.yaw = _yaw_from_quat(msg.pose.pose.orientation)

        if not self.pose_ready:
            self.start_x = self.x
            self.start_y = self.y
            self._init_mline()

        self.pose_ready = True

    def scan_cb(self, msg):
        r = msg.ranges

        self.front = min([v for v in r[310:410] if math.isfinite(v)] or [10.0])
        self.left  = min([v for v in r[540:720] if math.isfinite(v)] or [10.0])

        if self.pose_ready:
            self.step()

    # ── M-line ────────────────────────────────────────────────────────────────

    def _init_mline(self):
        dx = self.goal_x - self.start_x
        dy = self.goal_y - self.start_y

        self.mline_a   = dy
        self.mline_b   = -dx
        self.mline_c   = dx * self.start_y - dy * self.start_x
        self.mline_len = math.hypot(dx, dy)
        self.mline_ready = True

    def on_mline(self):
        if not self.mline_ready:
            return False

        dist = abs(self.mline_a * self.x + self.mline_b * self.y + self.mline_c) / self.mline_len
        return dist < MLINE_THRESH

    # ── Main loop ─────────────────────────────────────────────────────────────

    def step(self):
        twist = Twist()

        dist_to_goal = math.hypot(self.goal_x - self.x, self.goal_y - self.y)

        rospy.loginfo(
            f"[{self.state}] pos=({self.x:.2f},{self.y:.2f}) "
            f"front={self.front:.2f} left={self.left:.2f} "
            f"dist_goal={dist_to_goal:.2f}"
        )

        if dist_to_goal < GOAL_THRESHOLD:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)
            rospy.loginfo('Goal reached!')
            return

        # ✅ NEW: If goal is very close, ignore obstacles and go straight
        if dist_to_goal < OBSTACLE_DIST:
            # Override state – drive toward goal
            angle_to_goal = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
            angle_err = _norm_angle(angle_to_goal - self.yaw)
            twist.linear.x = LINEAR_SPD * 0.5
            twist.angular.z = angle_err * 1.5
            twist.angular.z = -twist.angular.z  # simulator sign
            self.pub.publish(twist)
            return

        if   self.state == ALIGN_TO_GOAL:   self._align(twist)
        elif self.state == MOVE_TO_GOAL:    self._move(twist)
        elif self.state == WALL_FOLLOWING:  self._wall_follow(twist, dist_to_goal)
        elif self.state == GO_REVERSE:      self._reverse(twist)
        elif self.state == GO_CLOSE:        self._go_close(twist)
        elif self.state == TURN_CORNER:     self._turn_corner(twist)

        twist.angular.z = -twist.angular.z  # simulator requirement
        self.pub.publish(twist)

    # ── State handlers ────────────────────────────────────────────────────────

    def _align(self, twist):
        angle_to_goal = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
        angle_err     = _norm_angle(angle_to_goal - self.yaw)

        if abs(angle_err) < ALIGN_THRESH:
            self.state = MOVE_TO_GOAL
            rospy.loginfo("Aligned → MOVE_TO_GOAL")
            return

        twist.angular.z = max(-ANGULAR_SPD, min(ANGULAR_SPD, angle_err * 2.0))

    def _move(self, twist):
        angle_to_goal = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
        angle_err     = _norm_angle(angle_to_goal - self.yaw)

        # Obstacle check – but only if goal is not too close (already handled above)
        if self.front < OBSTACLE_DIST:
            self.hit_point = (self.x, self.y)
            self.state = WALL_FOLLOWING
            rospy.loginfo("Obstacle → WALL_FOLLOWING")
            return

        if abs(angle_err) > math.radians(10):
            self.state = ALIGN_TO_GOAL
            return

        twist.linear.x  = LINEAR_SPD * 0.6
        twist.angular.z = angle_err * 1.5

    def _wall_follow(self, twist, dist_to_goal):
        angle_to_goal = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
        angle_err     = _norm_angle(angle_to_goal - self.yaw)

        # ── Leave condition (Bug2) – only if front is clear ──────────────
        if (self.on_mline() and self.hit_point is not None and
            self.front > OBSTACLE_DIST and abs(angle_err) < 0.3):
            hit_dist = math.hypot(
                self.goal_x - self.hit_point[0],
                self.goal_y - self.hit_point[1]
            )
            if dist_to_goal < hit_dist:
                self.state = ALIGN_TO_GOAL
                self.hit_point = None
                rospy.loginfo("Leave wall → ALIGN_TO_GOAL")
                return

        # ── Corner detection – but not if goal is the "obstacle" ──────────
        # (dist_to_goal already > OBSTACLE_DIST here, so safe)
        if self.front < OBSTACLE_DIST and self.left < WALL_DIST:
            if self.last_corner_finish_time is not None:
                elapsed_since_finish = (rospy.Time.now() - self.last_corner_finish_time).to_sec()
                if elapsed_since_finish < 2.0:
                    pass
                else:
                    self._start_corner()
            else:
                self._start_corner()
            return

        # Too close to front
        if self.front < CLOSE_FRONT_LIM:
            self.close_counter += 1
            if self.close_counter > CLOSE_COUNT_MAX:
                self.state = GO_REVERSE
                self.close_counter = 0
            else:
                twist.angular.z = -ANGULAR_SPD
            return

        self.close_counter = 0

        # Lost wall
        if self.left > FAR_WALL_LIMIT:
            self.far_from_wall_counter += 1
            if self.far_from_wall_counter > FAR_COUNT_MAX:
                self.state = GO_CLOSE
                self.far_from_wall_counter = 0
            else:
                twist.linear.x = LINEAR_SPD * 0.5
                twist.angular.z = ANGULAR_SPD * 0.5
            return

        self.far_from_wall_counter = 0

        # Normal PD wall following
        left_err = WALL_DIST - self.left
        d_err = left_err - self.prev_left_err
        self.prev_left_err = left_err

        kp, kd = 1.5, 1.0
        control = kp * left_err + kd * d_err

        twist.linear.x  = LINEAR_SPD * 0.8
        twist.angular.z = max(-ANGULAR_SPD, min(ANGULAR_SPD, control))

    def _start_corner(self):
        rospy.loginfo(f"Starting corner: front={self.front:.2f}, left={self.left:.2f}")
        self.state = TURN_CORNER
        self.turn_phase = 'reverse'
        self.turn_start_time = rospy.Time.now()

    def _turn_corner(self, twist):
        now = rospy.Time.now()
        elapsed = (now - self.turn_start_time).to_sec()

        if self.turn_phase == 'reverse':
            twist.linear.x = -LINEAR_SPD * 0.5
            twist.angular.z = 0.0
            if elapsed > 0.6:
                self.turn_phase = 'rotate'
                rospy.loginfo("Corner: reverse done, now rotating")
            return

        if self.turn_phase == 'rotate':
            twist.linear.x = 0.0
            if self.left < WALL_DIST:
                twist.angular.z = ANGULAR_SPD
            else:
                twist.angular.z = -ANGULAR_SPD

            if elapsed > 1.8:
                self.state = WALL_FOLLOWING
                self.turn_phase = None
                self.last_corner_finish_time = now
                rospy.loginfo("Corner: finished, back to wall following")

    def _reverse(self, twist):
        twist.linear.x = -LINEAR_SPD
        if self.front > 0.8:
            self.state = WALL_FOLLOWING
            rospy.loginfo("Recovered → WALL_FOLLOWING")

    def _go_close(self, twist):
        twist.linear.x  = LINEAR_SPD * 0.5
        twist.angular.z = ANGULAR_SPD * 0.5
        if self.left < FAR_WALL_LIMIT:
            self.state = WALL_FOLLOWING
            rospy.loginfo("Wall found → WALL_FOLLOWING")


if __name__ == '__main__':
    try:
        Navigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass