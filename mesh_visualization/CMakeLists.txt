cmake_minimum_required(VERSION 2.8.3)
project(mesh_visualization)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs roscpp
  topic_tools visualization_msgs)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp topic_tools visualization_msgs
  DEPENDS)

add_executable(mesh_visualization src/mesh_visualization.cpp)
add_dependencies(mesh_visualization ${catkin_EXPORTED_TARGETS})
target_link_libraries(mesh_visualization ${catkin_LIBRARIES})
