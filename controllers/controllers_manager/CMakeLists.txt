cmake_minimum_required(VERSION 2.8.3)
project(controllers_manager)

find_package(catkin REQUIRED COMPONENTS roscpp nodelet pluginlib quadrotor_msgs nav_msgs)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_service_files(
  FILES
  Transition.srv
)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
    DEPENDS roscpp nodelet pluginlib quadrotor_msgs nav_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include
    LIBRARIES # TODO
)

add_library(controllers_manager src/controllers_manager.cpp)

add_dependencies(controllers_manager quadrotor_msgs_generate_messages_cpp)