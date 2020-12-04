cmake_minimum_required(VERSION 3.10)
project(kr_tracker_msgs)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS message_generation nav_msgs geometry_msgs actionlib_msgs)

add_service_files(DIRECTORY srv FILES Transition.srv)

add_action_files(
  DIRECTORY
  action
  FILES
  LineTracker.action
  VelocityTracker.action
  CircleTracker.action
  TrajectoryTracker.action
  LissajousTracker.action
  LissajousAdder.action)

add_message_files(
  DIRECTORY
  msg
  FILES
  TrackerStatus.msg
  VelocityGoal.msg)

generate_messages(DEPENDENCIES geometry_msgs actionlib_msgs)

catkin_package(
  CATKIN_DEPENDS
  nav_msgs
  geometry_msgs
  actionlib_msgs
  message_runtime)
