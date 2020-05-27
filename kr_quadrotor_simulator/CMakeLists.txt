cmake_minimum_required(VERSION 2.8.3)
project(kr_quadrotor_simulator)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

find_package(catkin REQUIRED COMPONENTS roscpp kr_mav_msgs nav_msgs sensor_msgs tf2_ros)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp kr_mav_msgs nav_msgs sensor_msgs tf2_ros
  DEPENDS EIGEN3)

add_library(kr_quadrotor_dynamics src/dynamics/Quadrotor.cpp)

add_executable(kr_quadrotor_simulator_so3 src/quadrotor_simulator_so3.cpp)
target_link_libraries(kr_quadrotor_simulator_so3 kr_quadrotor_dynamics ${catkin_LIBRARIES})
add_dependencies(kr_quadrotor_simulator_so3 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(kr_quadrotor_simulator_trpy src/quadrotor_simulator_trpy.cpp)
target_link_libraries(kr_quadrotor_simulator_trpy kr_quadrotor_dynamics ${catkin_LIBRARIES})
add_dependencies(kr_quadrotor_simulator_trpy ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(TARGETS
    kr_quadrotor_dynamics
    kr_quadrotor_simulator_so3
    kr_quadrotor_simulator_trpy
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
