#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

echo "[run.sh] Generating randomised world..."
python3 "$(rospack find com760cw2_group6)/scripts/generate_world.py"

echo "[run.sh] Launching..."
roslaunch com760cw2_group6 group_6_launch.launch
