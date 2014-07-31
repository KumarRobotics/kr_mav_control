cmake_minimum_required(VERSION 2.8.3)
project(kquad_interface)

find_package(catkin REQUIRED COMPONENTS roscpp nodelet quadrotor_msgs sensor_msgs geometry_msgs)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS roscpp nodelet quadrotor_msgs sensor_msgs geometry_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

add_library(kquad_interface src/kQuadInterface.cc
                                     src/SerialDevice.cc
                                     src/kBotPacket2.c
                                     src/crc32.c
                                     src/kBotPacketParser.cc
                                     src/kQuadInterfaceDataTypes.cc)

add_library(kquad_interface_nodelet src/kquad_interface_nodelet.cpp)

add_dependencies(kquad_interface_nodelet quadrotor_msgs_generate_messages_cpp)

target_link_libraries(kquad_interface_nodelet kquad_interface)