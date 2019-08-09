cmake_minimum_required(VERSION 2.8.12)
project(trackers_msgs)

add_compile_options(-std=c++11)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs nav_msgs actionlib_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_service_files(DIRECTORY srv
  FILES
  Transition.srv
)

add_action_files(DIRECTORY action
  FILES
  LineTracker.action
  VelocityTracker.action
  CircleTracker.action
  TrajectoryTracker.action
  LissajousTracker.action
  LissajousAdder.action)

add_message_files(DIRECTORY msg
  FILES
  TrackerStatus.msg
)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs actionlib_msgs)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES encode_msgs decode_msgs
  CATKIN_DEPENDS message_runtime geometry_msgs nav_msgs
  DEPENDS)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})