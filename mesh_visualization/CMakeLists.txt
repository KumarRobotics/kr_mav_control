cmake_minimum_required(VERSION 2.8.3)
project(mesh_visualization)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs visualization_msgs)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp nav_msgs visualization_msgs
  DEPENDS)

add_executable(mesh_visualization src/mesh_visualization.cpp)
add_dependencies(mesh_visualization ${catkin_EXPORTED_TARGETS})
target_link_libraries(mesh_visualization ${catkin_LIBRARIES})
