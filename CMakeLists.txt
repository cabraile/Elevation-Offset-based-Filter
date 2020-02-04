cmake_minimum_required(VERSION 3.10)
project(elevation_offset_based_filter)

## Find catkin macros and libraries, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge 
  image_transport 
  sensor_msgs
  message_generation
  tf
)

## catkin specific configuration ##
catkin_package(
  CATKIN_DEPENDS
  message_runtime
  cv_bridge 
  image_transport
  sensor_msgs
)

find_package(OpenCV)

## Build ##
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(filter_node source/filter_node.cpp)
target_link_libraries(filter_node ${catkin_LIBRARIES})