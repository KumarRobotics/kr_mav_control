cmake_minimum_required(VERSION 2.8.12)
project(quadrotor_msgs)

add_compile_options(-std=c++11)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs rosconsole std_msgs)
find_package(Eigen3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_message_files(DIRECTORY msg
  FILES
  AuxCommand.msg
  Corrections.msg
  OutputData.msg
  PositionCommand.msg
  PWMCommand.msg
  Serial.msg
  SO3Command.msg
  StatusData.msg
  TRPYCommand.msg)

generate_messages(
  DEPENDENCIES geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES encode_msgs decode_msgs
  CATKIN_DEPENDS message_runtime geometry_msgs std_msgs
  DEPENDS EIGEN3)

add_library(encode_msgs src/encode_msgs.cpp)
add_dependencies(encode_msgs ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(encode_msgs ${catkin_LIBRARIES})

add_library(decode_msgs src/decode_msgs.cpp)
add_dependencies(decode_msgs ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(decode_msgs ${catkin_LIBRARIES})

install(TARGETS encode_msgs decode_msgs
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
