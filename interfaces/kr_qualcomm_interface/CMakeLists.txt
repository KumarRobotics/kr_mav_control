cmake_minimum_required(VERSION 3.10)
project(kr_qualcomm_interface)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             kr_mav_msgs
             nav_msgs
             nodelet
             tf_conversions)
find_package(Eigen3 REQUIRED)

find_package(Snav)
if(NOT ${Snav_FOUND})
  message(WARNING "NOTE: snav not found so not building kr_qualcomm_interface")
else()
  include_directories(${Snav_INCLUDE_DIR})
  include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

  catkin_package(
    LIBRARIES
    ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    kr_mav_msgs
    nav_msgs
    nodelet
    tf_conversions
    DEPENDS
    EIGEN3)

  add_library(${PROJECT_NAME} src/so3cmd_to_qualcomm_nodelet.cpp src/trpycmd_to_snav_nodelet.cpp
                              src/poscmd_to_snav_nodelet.cpp)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Snav_LIBRARY})

  add_executable(vio_odom_publisher src/vio_odom_publisher.cpp)
  target_link_libraries(vio_odom_publisher ${catkin_LIBRARIES} ${Snav_LIBRARY})

  add_executable(snav_publisher src/snav_publisher.cpp)
  add_dependencies(snav_publisher ${catkin_EXPORTED_TARGETS})
  target_link_libraries(snav_publisher ${catkin_LIBRARIES} ${Snav_LIBRARY})

  install(
    TARGETS ${PROJECT_NAME} vio_odom_publisher snav_publisher
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
  install(FILES nodelet_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

endif()
