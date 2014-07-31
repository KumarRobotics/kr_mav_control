cmake_minimum_required(VERSION 2.8.3)
project(so3_control)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs quadrotor_msgs std_msgs tf nodelet cmake_modules)
find_package(Eigen 3.0.0 REQUIRED)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIR})

catkin_package(
    DEPENDS eigen3
    CATKIN_DEPENDS roscpp nav_msgs quadrotor_msgs std_msgs tf nodelet
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

add_library(SO3Control src/SO3Control.cpp)

add_library(so3_control_nodelet src/so3_control_nodelet.cpp)
add_library(so3_trpy_control src/so3_trpy_control.cpp)

add_dependencies(so3_trpy_control quadrotor_msgs_generate_messages_cpp)

target_link_libraries(so3_control_nodelet SO3Control)
target_link_libraries(so3_trpy_control SO3Control)
target_link_libraries(so3_trpy_control ${catkin_LIBRARIES} )