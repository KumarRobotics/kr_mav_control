cmake_minimum_required(VERSION 3.10)
project(kr_mavros_interface)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             nav_msgs
             geometry_msgs
             kr_mav_msgs
             nodelet)
find_package(Eigen3 REQUIRED)

# If mavros_msgs not found, don't build but do not cause a build error
find_package(mavros_msgs)

if(NOT ${mavros_msgs_FOUND})
  message(WARNING "NOTE: mavros_msgs not found so not building kr_mavros_interface")
else()
  include_directories(${catkin_INCLUDE_DIRS} ${mavros_msgs_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

  catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    nav_msgs
    geometry_msgs
    kr_mav_msgs
    mavros_msgs
    nodelet
    DEPENDS
    EIGEN3)

  add_library(${PROJECT_NAME} src/so3cmd_to_mavros_nodelet.cpp)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

  install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  install(FILES nodelet_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif()
