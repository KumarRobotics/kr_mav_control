cmake_minimum_required(VERSION 2.8.3)
project(pid_control)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs quadrotor_msgs std_msgs nodelet tf cmake_modules)
find_package(Eigen REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp nav_msgs quadrotor_msgs std_msgs nodelet tf
  DEPENDS Eigen)

add_library(PIDControl src/PIDControl.cpp)

add_library(pid_control_nodelet src/pid_control_nodelet.cpp)
add_dependencies(pid_control_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(pid_control_nodelet PIDControl ${catkin_LIBRARIES})

install(FILES nodelet_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
