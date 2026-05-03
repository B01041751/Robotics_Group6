#!/bin/bash
cd /home/hashie/catkin_ws
catkin_make
python3 src/com760cw2_group6/scripts/generate_world.py
source src/com760cw2_group6/worlds/spawn_config.env
roslaunch com760cw2_group6 group_6_launch.launch
