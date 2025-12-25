cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_saver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_srvs
  message_generation
)

add_service_files(
  FILES
  SaveMap.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    rospy 
    sensor_msgs 
    std_srvs
    message_runtime
)

catkin_install_python(PROGRAMS
  scripts/pointcloud_saver_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
