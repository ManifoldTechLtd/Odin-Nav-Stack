cmake_minimum_required(VERSION 3.0.2)
project(navigation_planner)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  move_base
  dwa_local_planner
  navfn
  costmap_2d
  nav_core
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
)

## System dependencies
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
    move_base
    dwa_local_planner
    navfn
    costmap_2d
    nav_core
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf2
    tf2_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE)