cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_msgs)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs nav_msgs cmake_modules)
find_package(Eigen REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR})

add_message_files(DIRECTORY msg
  FILES
  AuxCommand.msg
  Corrections.msg
  FlatOutputs.msg
  OutputData.msg
  PositionCommand.msg
  PWMCommand.msg
  Serial.msg
  SO3Command.msg
  StatusData.msg
  TRPYCommand.msg)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES encode_msgs decode_msgs
  CATKIN_DEPENDS message_runtime geometry_msgs nav_msgs
  DEPENDS eigen)

add_library(encode_msgs src/encode_msgs.cpp)
add_dependencies(encode_msgs ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(encode_msgs ${catkin_LIBRARIES})

add_library(decode_msgs src/decode_msgs.cpp)
add_dependencies(decode_msgs ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(decode_msgs ${catkin_LIBRARIES})
