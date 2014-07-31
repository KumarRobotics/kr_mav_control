cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs cmake_modules)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen 3.0.0 REQUIRED)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR})

add_message_files(
  FILES
  AuxCommand.msg
  Corrections.msg
  OutputData.msg
  PositionCommand.msg
  Serial.msg
  SO3Command.msg
  StatusData.msg
  TRPYCommand.msg
)
generate_messages(
    DEPENDENCIES geometry_msgs std_msgs
)

catkin_package(
    DEPENDS eigen3
    CATKIN_DEPENDS geometry_msgs nav_msgs
    INCLUDE_DIRS include
    LIBRARIES # TODO
)

add_library(encode_msgs src/encode_msgs.cpp)
add_library(decode_msgs src/decode_msgs.cpp)

add_dependencies(encode_msgs quadrotor_msgs_generate_messages_cpp)
add_dependencies(decode_msgs quadrotor_msgs_generate_messages_cpp)
