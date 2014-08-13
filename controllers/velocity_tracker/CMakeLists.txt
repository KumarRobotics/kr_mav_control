cmake_minimum_required(VERSION 2.8.3)
project(velocity_tracker)

find_package(catkin REQUIRED COMPONENTS roscpp quadrotor_msgs nav_msgs geometry_msgs controllers_manager tf)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp quadrotor_msgs nav_msgs geometry_msgs controllers_manager tf
  DEPENDS)

add_library(velocity_tracker_plugin src/velocity_tracker_plugin.cpp)
add_dependencies(velocity_tracker_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocity_tracker_plugin ${catkin_LIBRARIES})
