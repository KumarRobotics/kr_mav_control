cmake_minimum_required(VERSION 3.10)
project(kr_mav_msgs)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs)
find_package(Eigen3 REQUIRED)

add_message_files(
  DIRECTORY
  msg
  FILES
  AuxCommand.msg
  Corrections.msg
  MotorRPM.msg
  OutputData.msg
  PWMCommand.msg
  PositionCommand.msg
  SO3Command.msg
  Serial.msg
  StatusData.msg
  TRPYCommand.msg)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  DEPENDS
  EIGEN3)
