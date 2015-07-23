cmake_minimum_required(VERSION 2.8.3)
project(super_modified_servo)
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
  Command.msg
)
## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)
###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES super_modified_servo
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
)
## Declare a cpp executable
add_executable(SuperModifiedTest src/SuperModifiedTest.cpp)
## Declare a cpp library
add_library(ZerooneSupermodified src/ZerooneSupermodified.c)
target_link_libraries(SuperModifiedTest ZerooneSupermodified ${catkin_LIBRARIES})