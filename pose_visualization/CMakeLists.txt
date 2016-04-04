cmake_minimum_required(VERSION 2.8.3)
project(pose_visualization)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

list(APPEND CATKIN_DEPENDENCIES geometry_msgs nav_msgs quadrotor_msgs
  roscpp tf topic_tools)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDENCIES})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  DEPENDS)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(pose_visualization src/pose_visualization.cpp)
add_dependencies(pose_visualization ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_visualization ${catkin_LIBRARIES})

install(TARGETS pose_visualization
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
