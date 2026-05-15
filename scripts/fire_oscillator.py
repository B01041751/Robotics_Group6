#!/usr/bin/env python3
import math
import os
import sys
import time

import rospy
import rospkg
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

RADIUS = 0.2
PERIOD = 2.0
FIRE_Z = 0.55
HZ     = 40


def load_fire_centres():
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
        print(f"[FireOsc] Could not load spawn_config.env: {e}")

    centres = []
    i = 1
    while True:
        fx = os.environ.get(f'FIRE{i}_X')
        fy = os.environ.get(f'FIRE{i}_Y')
        if fx is None or fy is None:
            break
        centres.append((f'fire_{i}', float(fx), float(fy)))
        i += 1
    if not centres:
        centres = [('fire_1', -2.0, 1.5), ('fire_2', 3.5, -3.0)]
    return centres


def main():
    rospy.init_node('fire_oscillator', anonymous=True)

    print("[FireOsc] Waiting for /gazebo/set_model_state ...")
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    fires = load_fire_centres()
    print(f"[FireOsc] Oscillating {len(fires)} fire(s) — radius={RADIUS}m  period={PERIOD}s")
    for name, cx, cy in fires:
        print(f"  {name} centred at ({cx:.2f}, {cy:.2f})")

    t  = 0.0
    dt = 1.0 / HZ

    while not rospy.is_shutdown():
        for name, cx, cy in fires:
            angle = 2.0 * math.pi * t / PERIOD
            x = cx + RADIUS * math.cos(angle)
            y = cy + RADIUS * math.sin(angle)

            state = ModelState()
            state.model_name         = name
            state.pose.position.x    = x
            state.pose.position.y    = y
            state.pose.position.z    = FIRE_Z
            state.pose.orientation.w = 1.0
            state.twist.linear.x     = 0.0
            state.twist.linear.y     = 0.0
            state.twist.linear.z     = 0.0
            state.twist.angular.x    = 0.0
            state.twist.angular.y    = 0.0
            state.twist.angular.z    = 0.0
            state.reference_frame    = 'world'

            try:
                resp = set_state(state)
                if not resp.success:
                    rospy.logwarn_throttle(5.0, f"[FireOsc] {name}: {resp.status_message}")
            except rospy.ServiceException as e:
                rospy.logwarn_throttle(5.0, f"[FireOsc] service error: {e}")

        t += dt
        time.sleep(dt)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
