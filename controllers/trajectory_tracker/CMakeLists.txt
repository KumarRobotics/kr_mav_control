cmake_minimum_required(VERSION 2.8.3)
project(trajectory_tracker)

add_definitions("-std=c++11")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  cmake_modules
)


if(CMAKE_SIZEOF_VOID_P EQUAL 8) 
  set(LINUX_BIT 64)
else()
  set(LINUX_BIT 32)
endif()

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV 2.4.6 REQUIRED)

if(LINUX_BIT EQUAL 64)
	find_package(GUROBI REQUIRED)
	include_directories(${GUROBI_INCLUDE_DIRS})
endif()

include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES trajectory_tracker
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
add_executable(trajectory_base src/trajectory_base.cpp)
target_link_libraries(trajectory_base
  ${catkin_LIBRARIES}
  ${GUROBI_CXX_LIBRARY}
  ${GUROBI_LIBRARY}
)

add_executable(trajectory_generator src/traj_gen.cpp)
target_link_libraries(trajectory_generator
  ${catkin_LIBRARIES}
)
