cmake_minimum_required(VERSION 2.8.3)
project(null_tracker)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp quadrotor_msgs nodelet controllers_manager
  cmake_modules)
find_package(Eigen REQUIRED)

include_directories(include ${EIGEN_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp quadrotor_msgs nodelet controllers_manager
  DEPENDS Eigen)

add_library(null_tracker src/null_tracker.cpp)
add_dependencies(null_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(null_tracker ${catkin_LIBRARIES})
