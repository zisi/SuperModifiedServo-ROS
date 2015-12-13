cmake_minimum_required(VERSION 2.8.3)
project(super_modified_servo)
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  dynamic_reconfigure
  sensor_msgs
)
################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
)
## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(cfg/commander.cfg)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ZerooneSupermodified
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)
###########
## Build ##
###########
## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  cfg/cpp
)
## Declare a cpp executable
add_executable(Commander src/Commander.cpp)
## Declare a cpp library
add_library(ZerooneSupermodified src/ZerooneSupermodified.c)
target_link_libraries(Commander ZerooneSupermodified ${catkin_LIBRARIES})
## make sure configure headers are built before any node using them
add_dependencies(Commander ${PROJECT_NAME}_gencfg)
