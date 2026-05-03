#!/usr/bin/env python3
"""Monitors robot odometry and spawns a closing wall at each entrance once the
robot has moved fully inside the arena boundary."""
import threading
import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest

INSIDE_THRESHOLD = 8.5   # robot is "inside" when |x| < this AND |y| < this


def _wall_sdf(name, x, y, sx, sy):
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <pose>{x:.4f} {y:.4f} 0.5 0 0 0</pose>
    <link name="link">
      <collision name="c">
        <geometry><box><size>{sx} {sy} 1.0</size></box></geometry>
      </collision>
      <visual name="v">
        <cast_shadows>false</cast_shadows>
        <geometry><box><size>{sx} {sy} 1.0</size></box></geometry>
        <material>
          <ambient>0.55 0.5 0.45 1</ambient>
          <diffuse>0.55 0.5 0.45 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


class EntranceManager:
    def __init__(self):
        rospy.init_node('entrance_manager')

        self._closed = {'bot0': False, 'bot1': False}
        self._lock = threading.Lock()

        # Entrance wall centre positions and sides, set from launch params
        self._cfg = {
            'bot0': {
                'side':  rospy.get_param('~bot0_side',  'S'),
                'ent_x': float(rospy.get_param('~bot0_ent_x', 0.0)),
                'ent_y': float(rospy.get_param('~bot0_ent_y', -10.0)),
            },
            'bot1': {
                'side':  rospy.get_param('~bot1_side',  'W'),
                'ent_x': float(rospy.get_param('~bot1_ent_x', -10.0)),
                'ent_y': float(rospy.get_param('~bot1_ent_y', 0.0)),
            },
        }

        rospy.loginfo('[EntranceManager] Waiting for /gazebo/spawn_sdf_model ...')
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=5.0)
                break
            except rospy.ROSException:
                rospy.logwarn('[EntranceManager] Gazebo not ready yet, retrying...')
        self._spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        rospy.Subscriber('/Group6Bot_0/odom', Odometry,
                         lambda msg: self._check(msg, 'bot0'))
        rospy.Subscriber('/Group6Bot_1/odom', Odometry,
                         lambda msg: self._check(msg, 'bot1'))

        rospy.loginfo('[EntranceManager] Watching for robots to enter the arena...')

    def _check(self, msg, bot_id):
        with self._lock:
            if self._closed[bot_id]:
                return
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            if abs(px) < INSIDE_THRESHOLD and abs(py) < INSIDE_THRESHOLD:
                self._closed[bot_id] = True
                threading.Thread(target=self._close, args=(bot_id,), daemon=True).start()

    def _close(self, bot_id):
        cfg = self._cfg[bot_id]
        side  = cfg['side']
        ex, ey = cfg['ent_x'], cfg['ent_y']
        name  = f'closing_wall_{bot_id}'

        # Wall orientation matches the entrance side
        if side in ('N', 'S'):
            sx, sy = 1.2, 0.2   # gap runs along x
        else:
            sx, sy = 0.2, 1.2   # gap runs along y

        sdf = _wall_sdf(name, ex, ey, sx, sy)
        req = SpawnModelRequest()
        req.model_name = name
        req.model_xml  = sdf
        try:
            self._spawn(req)
            rospy.loginfo(f'[EntranceManager] Entrance closed for {bot_id} '
                          f'(side={side}, x={ex:.2f}, y={ey:.2f})')
        except Exception as exc:
            rospy.logerr(f'[EntranceManager] Failed to close entrance for {bot_id}: {exc}')

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    EntranceManager().run()
