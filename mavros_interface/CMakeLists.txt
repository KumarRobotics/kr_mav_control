cmake_minimum_required(VERSION 2.8.3)
project(mavros_interface)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs geometry_msgs quadrotor_msgs tf eigen_conversions nodelet cmake_modules)
find_package(Eigen3 REQUIRED)

# If mavros_msgs not found, don't build but do not cause a build error
find_package(mavros_msgs)

if(NOT ${mavros_msgs_FOUND})
  message(WARNING "NOTE: mavros_msgs not found so not building mavros_interface")
else()
  include_directories(include ${catkin_INCLUDE_DIRS} ${mavros_msgs_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
    CATKIN_DEPENDS roscpp nav_msgs geometry_msgs quadrotor_msgs mavros_msgs tf eigen_conversions nodelet
    DEPENDS Eigen)

  add_library(mavros_interface_nodelet src/mavros_interface_nodelet.cpp)
  add_dependencies(mavros_interface_nodelet ${catkin_EXPORTED_TARGETS})
  target_link_libraries(mavros_interface_nodelet ${catkin_LIBRARIES})

  add_executable(attitude_tuning_tool src/attitude_tuning_tool.cpp)
  add_dependencies(attitude_tuning_tool ${catkin_EXPORTED_TARGETS})
  target_link_libraries(attitude_tuning_tool ${catkin_LIBRARIES})

  install(TARGETS mavros_interface_nodelet attitude_tuning_tool
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

  install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
  install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  install(FILES nodelet_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif()
