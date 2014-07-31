cmake_minimum_required(VERSION 2.8.3)
project(pid_control)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs quadrotor_msgs std_msgs nodelet tf)
find_package(Eigen REQUIRED)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIR})

catkin_package(
    DEPENDS roscpp nav_msgs quadrotor_msgs std_msgs nodelet tf
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

add_library(PIDControl src/PIDControl.cpp)
add_library(pid_control_nodelet src/pid_control_nodelet.cpp)

add_dependencies(pid_control_nodelet quadrotor_msgs_generate_messages_cpp)

target_link_libraries(pid_control_nodelet PIDControl)
target_link_libraries(pid_control_nodelet ${catkin_LIBRARIES})