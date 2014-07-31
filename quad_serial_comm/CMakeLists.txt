cmake_minimum_required(VERSION 2.8.3)
project(quad_serial_comm)

find_package(catkin REQUIRED COMPONENTS roscpp quadrotor_msgs asio-serial-device nodelet)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS roscpp quadrotor_msgs asio-serial-device nodelet
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

add_library(quad_serial_comm_nodelet src/quad_serial_comm_nodelet.cpp src/asctec_serial_interface.cpp)

add_dependencies(quad_serial_comm_nodelet quadrotor_msgs_generate_messages_cpp)
  
