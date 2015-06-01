cmake_minimum_required(VERSION 2.8.3)
project(trajectory_trackers)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS geometry_msgs tf trackers_manager cmake_modules)
find_package(Eigen REQUIRED)

include_directories(include ${EIGEN_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS geometry_msgs tf trackers_manager
  DEPENDS Eigen)

add_library(position_command_tracker src/position_command_tracker.cpp)
add_dependencies(position_command_tracker ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(position_command_tracker ${catkin_LIBRARIES})
