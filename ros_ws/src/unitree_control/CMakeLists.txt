cmake_minimum_required(VERSION 3.0.2)
project(unitree_control)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Threads REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf2
  tf2_ros
  std_srvs
)
find_package(PkgConfig REQUIRED)
catkin_package()
include_directories(
  /usr/local/include/unitree
  ${catkin_INCLUDE_DIRS}
)
link_directories(
  ${catkin_LIBRARY_DIRS}
  /usr/local/lib
)
add_executable(unitree_vel_controller 
  src/unitree_vel_controller.cpp
)
add_dependencies(unitree_vel_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(unitree_vel_controller
  ${catkin_LIBRARIES}
  ddsc ddscxx
  Threads::Threads
  unitree_sdk2
)


