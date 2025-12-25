cmake_minimum_required(VERSION 3.0.2)
project(pcd2pgm)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  nav_msgs
  roslib
)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(pcd2pgm_node src/pcd2pgm_node.cpp)
target_link_libraries(pcd2pgm_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

