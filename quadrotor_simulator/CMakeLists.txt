cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_simulator)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

find_package(catkin REQUIRED COMPONENTS roscpp quadrotor_msgs nav_msgs sensor_msgs tf2_ros cmake_modules)
find_package(Eigen REQUIRED)

include_directories(include ${EIGEN_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp quadrotor_msgs nav_msgs sensor_msgs tf2_ros
  DEPENDS Eigen)

add_library(quadrotor_dynamics src/dynamics/Quadrotor.cpp)

add_executable(quadrotor_simulator_so3 src/quadrotor_simulator_so3.cpp)
target_link_libraries(quadrotor_simulator_so3 quadrotor_dynamics ${catkin_LIBRARIES})
add_dependencies(quadrotor_simulator_so3 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(quadrotor_simulator_trpy src/quadrotor_simulator_trpy.cpp)
target_link_libraries(quadrotor_simulator_trpy quadrotor_dynamics ${catkin_LIBRARIES})
add_dependencies(quadrotor_simulator_trpy ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(TARGETS
    quadrotor_dynamics
    quadrotor_simulator_so3
    quadrotor_simulator_trpy
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
