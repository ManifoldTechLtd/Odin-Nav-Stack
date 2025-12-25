cmake_minimum_required(VERSION 3.0.2)
project(fish2pinhole)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
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

add_executable(fish2pinhole_node src/fish2pinhole_node.cpp)
target_link_libraries(fish2pinhole_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(cloud_crop_node src/cloud_crop_node.cpp)
target_link_libraries(cloud_crop_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


