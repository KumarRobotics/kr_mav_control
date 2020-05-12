cmake_minimum_required(VERSION 2.8.3)
project(qc_serial_interface)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

find_package(catkin REQUIRED COMPONENTS nodelet quadrotor_msgs roscpp sensor_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS nodelet quadrotor_msgs roscpp sensor_msgs
  DEPENDS)

add_library(quad_encode_msg_nodelet src/encode_msgs.cpp src/quad_encode_msg_nodelet.cpp)
add_dependencies(quad_encode_msg_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(quad_encode_msg_nodelet ${catkin_LIBRARIES})

add_library(quad_decode_msg_nodelet src/decode_msgs.cpp src/quad_decode_msg_nodelet.cpp)
add_dependencies(quad_decode_msg_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(quad_decode_msg_nodelet ${catkin_LIBRARIES})

add_library(quad_serial_comm_nodelet src/ASIOSerialDevice.cc src/asctec_serial_interface.cpp src/quad_serial_comm_nodelet.cpp)
add_dependencies(quad_serial_comm_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(quad_serial_comm_nodelet ${catkin_LIBRARIES})

install(TARGETS
    quad_decode_msg_nodelet quad_encode_msg_nodelet quad_serial_comm_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES nodelet_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
