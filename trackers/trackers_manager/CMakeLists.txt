cmake_minimum_required(VERSION 2.8.3)
project(trackers_manager)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp nodelet pluginlib quadrotor_msgs nav_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_service_files(DIRECTORY srv
  FILES
  Transition.srv
)
generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trackers_manager
  CATKIN_DEPENDS roscpp nodelet pluginlib quadrotor_msgs nav_msgs
  DEPENDS
)

add_library(trackers_manager src/trackers_manager.cpp)
add_dependencies(trackers_manager ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(trackers_manager ${catkin_LIBRARIES})
