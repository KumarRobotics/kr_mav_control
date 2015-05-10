cmake_minimum_required(VERSION 2.8.3)
project(null_tracker)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS controllers_manager)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES null_tracker
  CATKIN_DEPENDS controllers_manager)

add_library(null_tracker src/null_tracker.cpp)
add_dependencies(null_tracker ${catkin_EXPORTED_TARGETS})
target_link_libraries(null_tracker ${catkin_LIBRARIES})
