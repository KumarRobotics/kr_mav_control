cmake_minimum_required(VERSION 3.10)
project(kr_serial_interface)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall)

find_package(catkin REQUIRED COMPONENTS nodelet kr_mav_msgs roscpp sensor_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  nodelet
  kr_mav_msgs
  roscpp
  sensor_msgs
  DEPENDS)

add_library(
  ${PROJECT_NAME}
  src/ASIOSerialDevice.cc
  src/asctec_serial_interface.cpp
  src/decode_msgs.cpp
  src/encode_msgs.cpp
  src/quad_decode_msg_nodelet.cpp
  src/quad_encode_msg.cpp
  src/quad_encode_msg_nodelet.cpp
  src/quad_serial_comm_nodelet.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES nodelet_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
