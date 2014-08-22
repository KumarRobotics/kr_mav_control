cmake_minimum_required(VERSION 2.8.3)
project(quad_encode_msg)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp quadrotor_msgs nodelet)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp quadrotor_msgs nodelet
  DEPENDS)

add_library(quad_encode_msg_nodelet src/quad_encode_msg_nodelet.cpp)
add_dependencies(quad_encode_msg_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(quad_encode_msg_nodelet ${catkin_LIBRARIES})
