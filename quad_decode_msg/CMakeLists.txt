cmake_minimum_required(VERSION 2.8.3)
project(quad_decode_msg)

find_package(catkin REQUIRED COMPONENTS roscpp quadrotor_msgs sensor_msgs nodelet)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp quadrotor_msgs sensor_msgs nodelet
  DEPENDS)

add_library(quad_decode_msg_nodelet src/quad_decode_msg_nodelet.cpp)
add_dependencies(quad_decode_msg_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(quad_decode_msg_nodelet ${catkin_LIBRARIES})
