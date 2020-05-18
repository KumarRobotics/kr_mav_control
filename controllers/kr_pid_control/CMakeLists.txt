cmake_minimum_required(VERSION 2.8.3)
project(kr_pid_control)

message(FATAL_ERROR "This package has not been updated for the the changes in \
the PositionCommand message and does not work. Please use the controllers in \
the kr_so3_control package!")

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs kr_mav_msgs std_msgs nodelet tf)
find_package(Eigen3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp nav_msgs kr_mav_msgs std_msgs nodelet tf
  DEPENDS EIGEN3)

add_library(PIDControl src/PIDControl.cpp)

add_library(kr_pid_control_nodelet src/kr_pid_control_nodelet.cpp)
add_dependencies(kr_pid_control_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(kr_pid_control_nodelet PIDControl ${catkin_LIBRARIES})

install(TARGETS
    PIDControl
    kr_pid_control_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES nodelet_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
