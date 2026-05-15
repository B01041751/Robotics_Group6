#!/bin/bash
export DISABLE_ROS1_EOL_WARNINGS=1
cd /home/hashie/catkin_ws
source src/com760cw2_group6/worlds/spawn_config.env

echo "[bot1] Spawning Bot 1 and starting Bug2 → Gas Zone 1 (${GAS1_X}, ${GAS1_Y})"
roslaunch com760cw2_group6 bot1_demo.launch
