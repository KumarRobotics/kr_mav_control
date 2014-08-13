cmake_minimum_required(VERSION 2.8.3)
project(quad_serial_comm)

find_package(catkin REQUIRED COMPONENTS roscpp quadrotor_msgs asio-serial-device nodelet)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp quadrotor_msgs asio-serial-device nodelet
  DEPENDS)

add_library(quad_serial_comm_nodelet src/quad_serial_comm_nodelet.cpp src/asctec_serial_interface.cpp)
add_dependencies(quad_serial_comm_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(quad_serial_comm_nodelet ${catkin_LIBRARIES})
