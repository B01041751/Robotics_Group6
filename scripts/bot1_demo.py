#!/usr/bin/env python3
import math
import os
import sys

import rospy
import roslaunch
import rospkg
from gazebo_msgs.srv import DeleteModel
from nav_msgs.msg import Odometry


def _load_spawn_config():
    try:
        pkg_path = rospkg.RosPack().get_path('com760cw2_group6')
        env_file = os.path.join(pkg_path, 'worlds', 'spawn_config.env')
        with open(env_file) as f:
            for line in f:
                line = line.strip()
                if line.startswith('export '):
                    line = line[7:]
                if '=' in line and not line.startswith('#'):
                    key, _, val = line.partition('=')
                    os.environ.setdefault(key.strip(), val.strip())
    except Exception as e:
        print(f'[bot1] WARNING: Could not load spawn_config.env: {e}')


def _delete_existing():
    try:
        rospy.wait_for_service('/gazebo/delete_model', timeout=5.0)
        svc  = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp = svc('Group6Bot_1')
        if resp.success:
            print('[bot1] Removed previous Bot 1 from Gazebo')
        rospy.sleep(0.5)
    except Exception:
        pass


def main():
    _load_spawn_config()

    goal_x = float(os.environ.get('GAS1_X', '0.0'))
    goal_y = float(os.environ.get('GAS1_Y', '0.0'))

    print('\n' + '=' * 60)
    print('  DEMO: Bot 1 — Bug2 navigation')
    print(f'  Spawn : ({os.environ.get("BOT1_X","?")} , {os.environ.get("BOT1_Y","?")})')
    print(f'  Goal  : Gas Zone 1 ({goal_x:.2f}, {goal_y:.2f})')
    print('=' * 60 + '\n')

    rospy.init_node('bot1_demo_monitor', anonymous=True)

    _delete_existing()

    pkg_path    = rospkg.RosPack().get_path('com760cw2_group6')
    launch_file = os.path.join(pkg_path, 'launch', 'bot1_demo.launch')

    uuid   = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [(launch_file, [])])
    launch.start()

    pos = [None]

    def _odom_cb(msg):
        pos[0] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    rospy.Subscriber('/Group6Bot_1/odom', Odometry, _odom_cb)

    print('[bot1] Navigating... (Ctrl-C to stop)\n')

    try:
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if pos[0] is not None:
                dist = math.hypot(goal_x - pos[0][0], goal_y - pos[0][1])
                if dist < 0.3:
                    print(f'\n[bot1] Goal reached! dist={dist:.3f} m')
                    break
            rate.sleep()
    except (KeyboardInterrupt, SystemExit):
        print('\n[bot1] Stopped')
    finally:
        launch.shutdown()
        if not rospy.is_shutdown():
            rospy.signal_shutdown('Bot 1 demo complete')

    print('[bot1] Done\n')


if __name__ == '__main__':
    main()
