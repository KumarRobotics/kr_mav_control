cmake_minimum_required(VERSION 3.10)
project(kr_mav_controllers)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_STANDARD 11)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             std_msgs
             nav_msgs
             geometry_msgs
             kr_mav_msgs
             tf
             nodelet
             dynamic_reconfigure)
find_package(Eigen3 REQUIRED NO_MODULE)

generate_dynamic_reconfigure_options(cfg/SO3.cfg)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  tf
  roscpp
  nodelet
  std_msgs
  nav_msgs
  kr_mav_msgs
  dynamic_reconfigure
  DEPENDS
  EIGEN3)

add_library(kr_mav_so3_control src/SO3Control.cpp src/so3_control_nodelet.cpp src/so3_trpy_control.cpp)
target_include_directories(kr_mav_so3_control PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(kr_mav_so3_control PUBLIC ${catkin_LIBRARIES} Eigen3::Eigen)
add_dependencies(kr_mav_so3_control ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

# TODO: PID control has not been updated for new messages add_library(kr_mav_pid_control src/PIDControl.cpp
# src/pid_control_nodelet.cpp) add_dependencies(kr_mav_pid_control ${catkin_EXPORTED_TARGETS})
# target_link_libraries(kr_mav_pid_control ${catkin_LIBRARIES})

install(
  TARGETS kr_mav_so3_control
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES nodelet_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
