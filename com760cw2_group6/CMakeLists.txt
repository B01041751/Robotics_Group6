cmake_minimum_required(VERSION 3.0.2)
project(com760cw2_group6)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  xacro
  gazebo_ros
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf xacro gazebo_ros
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
# Added hazard_train.py and hazard_env.py to the installation list
catkin_install_python(PROGRAMS
  scripts/moving_fireballs.py
  scripts/mission_node_group6.py
  scripts/hazard_train.py
  scripts/hazard_env.py
  scripts//hazard_oscillator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install resource folders
# This ensures your .urdf and .world files are copied to the share folder
install(DIRECTORY launch urdf worlds media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
