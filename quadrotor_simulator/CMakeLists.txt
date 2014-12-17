cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_simulator)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp quadrotor_msgs nav_msgs sensor_msgs tf2_ros cmake_modules)
find_package(Eigen REQUIRED)

include_directories(include include/odeint-v2/include ${EIGEN_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp quadrotor_msgs nav_msgs sensor_msgs tf2_ros
  DEPENDS Eigen)

add_library(quadrotor_dynamics src/dynamics/Quadrotor.cpp)

add_executable(quadrotor_simulator_so3 src/quadrotor_simulator_so3.cpp)
target_link_libraries(quadrotor_simulator_so3 quadrotor_dynamics ${catkin_LIBRARIES})

add_executable(quadrotor_simulator_trpy src/quadrotor_simulator_trpy.cpp)
target_link_libraries(quadrotor_simulator_trpy quadrotor_dynamics ${catkin_LIBRARIES})
