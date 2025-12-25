cmake_minimum_required(VERSION 3.0.2)
project(odin_vlm_terminal)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/ros_vlm_terminal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
