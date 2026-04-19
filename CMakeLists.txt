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
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   QLearning.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   SetBugStatus.srv
   Group6GoToPoint.srv
   Group6WallFollow.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf xacro gazebo_ros message_runtime
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
  scripts/group6bot_0_main.py
  scripts/group6bot_0_subscriber.py
  scripts/hazard_train.py
  scripts/hazard_env.py
  scripts/hazard_oscillator.py
  scripts/bug_service_server.py
  scripts/bug_service_client.py
  scripts/Group6Bot_1_explore.py
  scripts/Group6Bot_1_camera_view.py
  scripts/move_robot2_circle.py
  scripts/q_learning_rewards.py
  scripts/group6_monitor.py
  scripts/generate_world.py
  scripts/tunnel_flicker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install resource folders
# This ensures your .urdf and .world files are copied to the share folder
install(DIRECTORY launch urdf worlds media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
