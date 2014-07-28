# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(so3_control)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs quadrotor_msgs std_msgs tf nodelet)
find_package(Eigen3 REQUIRED)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})


add_library(SO3Control src/SO3Control.cpp)

# CATKIN_MIGRATION
# use link_directories() include_directories(), add_definitions(), target_link_libraries() or set_target_properties
# rosbuild_add_link_flags(SO3Control -Wl,--as-needed)

add_library(so3_control_nodelet src/so3_control_nodelet.cpp)
target_link_libraries(so3_control_nodelet SO3Control)

# CATKIN_MIGRATION
# use link_directories() include_directories(), add_definitions(), target_link_libraries() or set_target_properties
# rosbuild_add_link_flags(so3_control_nodelet -Wl,--as-needed)

add_library(so3_trpy_control src/so3_trpy_control.cpp)
target_link_libraries(so3_trpy_control SO3Control)

add_dependencies(so3_trpy_control quadrotor_msgs_generate_messages_cpp)
target_link_libraries(so3_trpy_control ${catkin_LIBRARIES} )


catkin_package(
    DEPENDS roscpp nav_msgs quadrotor_msgs std_msgs tf nodelet
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)