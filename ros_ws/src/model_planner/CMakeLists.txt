cmake_minimum_required(VERSION 3.0.2)
project(model_planner)


set(CMAKE_CXX_FLAGS "-std=c++14")
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
# Prefer Config-mode find and imported target for Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  tf2
  tf2_ros
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES global_planner local_planner
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs sensor_msgs tf tf2 tf2_ros
  DEPENDS Boost OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Global Planner Library
add_library(global_planner
  src/global_planner/global_planner.cpp
  src/global_planner/astar.cpp
  src/global_planner/costmap.cpp
)
target_link_libraries(global_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} Eigen3::Eigen)

# Local Planner Library
add_library(local_planner
  src/local_planner/local_planner.cpp
  src/local_planner/local_costmap.cpp
  src/local_planner/dwa_planner.cpp
)
target_link_libraries(local_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} Eigen3::Eigen)

# Global Planner Node
add_executable(global_planner_node src/global_planner/global_planner_node.cpp)
target_link_libraries(global_planner_node global_planner ${catkin_LIBRARIES})

# Local Planner Node
add_executable(local_planner_node src/local_planner/local_planner_node.cpp)
target_link_libraries(local_planner_node local_planner ${catkin_LIBRARIES})

# Install
# install(TARGETS global_planner_node local_planner_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
