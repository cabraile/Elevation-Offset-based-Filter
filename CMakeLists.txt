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
#  INCLUDE_DIRS include
#  LIBRARIES package_name
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS
  message_runtime
  cv_bridge 
  image_transport
  sensor_msgs
)

find_package(OpenCV)

## Build ##
## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable, the .cpp is what the executable program need
add_executable(relative_altitude_filter_node source/relative_altitude_filter_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(relative_altitude_filter_node ${catkin_LIBRARIES})