cmake_minimum_required(VERSION 2.8.3)
project(controllers_manager)

find_package(catkin REQUIRED COMPONENTS roscpp nodelet pluginlib quadrotor_msgs nav_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_service_files(DIRECTORY srv
  FILES
  Transition.srv
)
generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp nodelet pluginlib quadrotor_msgs nav_msgs
  DEPENDS)

add_library(controllers_manager src/controllers_manager.cpp)
add_dependencies(controllers_manager ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(controllers_manager ${catkin_LIBRARIES})
