cmake_minimum_required(VERSION 2.8.3)
project(quad_serial_comm)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp quadrotor_msgs asio_serial_device nodelet)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp quadrotor_msgs asio_serial_device nodelet
  DEPENDS)

add_library(quad_serial_comm_nodelet src/quad_serial_comm_nodelet.cpp src/asctec_serial_interface.cpp)
add_dependencies(quad_serial_comm_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(quad_serial_comm_nodelet ${catkin_LIBRARIES})

install(FILES nodelet_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
