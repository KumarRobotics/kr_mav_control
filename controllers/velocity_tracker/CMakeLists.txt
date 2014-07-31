cmake_minimum_required(VERSION 2.8.3)
project(velocity_tracker)

find_package(catkin REQUIRED COMPONENTS roscpp quadrotor_msgs nav_msgs geometry_msgs controllers_manager tf)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS roscpp quadrotor_msgs nav_msgs geometry_msgs controllers_manager tf
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
add_library(velocity_tracker_plugin src/velocity_tracker_plugin.cpp)

add_dependencies(velocity_tracker_plugin quadrotor_msgs_generate_messages_cpp)