cmake_minimum_required(VERSION 2.8.3)
project(kquad_interface)

find_package(catkin REQUIRED COMPONENTS roscpp nodelet quadrotor_msgs sensor_msgs geometry_msgs)
find_package(Threads REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp nodelet quadrotor_msgs sensor_msgs geometry_msgs
  DEPENDS)

add_library(kquad_interface src/kQuadInterface.cc
  src/SerialDevice.cc
  src/kBotPacket2.c
  src/crc32.c
  src/kBotPacketParser.cc
  src/kQuadInterfaceDataTypes.cc)
target_link_libraries(kquad_interface ${CMAKE_THREAD_LIBS_INIT})

add_library(kquad_interface_nodelet src/kquad_interface_nodelet.cpp)
target_link_libraries(kquad_interface_nodelet kquad_interface ${catkin_LIBRARIES})
