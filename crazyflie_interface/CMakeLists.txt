cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_interface)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs geometry_msgs  quadrotor_msgs nodelet cmake_modules)
find_package(Eigen 3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs quadrotor_msgs  nodelet
  DEPENDS Eigen)

add_library(so3cmd_to_crazyflie_nodelet src/so3cmd_to_crazyflie_nodelet.cpp)
add_dependencies(so3cmd_to_crazyflie_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(so3cmd_to_crazyflie_nodelet ${catkin_LIBRARIES})

install(TARGETS so3cmd_to_crazyflie_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(FILES nodelet_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
