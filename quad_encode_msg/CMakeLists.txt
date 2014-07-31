cmake_minimum_required(VERSION 2.8.3)
project(quad_encode_msg)

find_package(catkin REQUIRED COMPONENTS roscpp quadrotor_msgs nodelet)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS roscpp quadrotor_msgs nodelet
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

add_library(quad_encode_msg_nodelet src/quad_encode_msg_nodelet.cpp)

add_dependencies(quad_encode_msg_nodelet quadrotor_msgs_generate_messages_cpp)

target_link_libraries(quad_encode_msg_nodelet ${catkin_LIBRARIES})