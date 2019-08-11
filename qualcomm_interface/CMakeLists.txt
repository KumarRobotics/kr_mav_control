cmake_minimum_required(VERSION 2.8.3)
project(qualcomm_interface)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

find_package(catkin REQUIRED COMPONENTS roscpp quadrotor_msgs nav_msgs nodelet tf_conversions)
find_package(Eigen3 REQUIRED)

find_package(Snav)
if(NOT ${Snav_FOUND})
  message(WARNING "NOTE: snav not found so not building qualcomm_interface")
else()
  include_directories(${Snav_INCLUDE_DIR})
  include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

  catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS roscpp quadrotor_msgs nav_msgs nodelet tf_conversions
    DEPENDS EIGEN3)

  add_executable(vio_odom_publisher src/vio_odom_publisher.cpp)
  target_link_libraries(vio_odom_publisher ${catkin_LIBRARIES} ${Snav_LIBRARY})

  add_library(so3cmd_to_qualcomm_nodelet src/so3cmd_to_qualcomm_nodelet.cpp)
  add_dependencies(so3cmd_to_qualcomm_nodelet ${catkin_EXPORTED_TARGETS})
  target_link_libraries(so3cmd_to_qualcomm_nodelet ${catkin_LIBRARIES} ${Snav_LIBRARY})

  add_library(trpycmd_to_snav_nodelet src/trpycmd_to_snav_nodelet.cpp)
  add_dependencies(trpycmd_to_snav_nodelet ${catkin_EXPORTED_TARGETS})
  target_link_libraries(trpycmd_to_snav_nodelet ${catkin_LIBRARIES} ${Snav_LIBRARY})

  add_library(poscmd_to_snav_nodelet src/poscmd_to_snav_nodelet.cpp)
  add_dependencies(poscmd_to_snav_nodelet ${catkin_EXPORTED_TARGETS})
  target_link_libraries(poscmd_to_snav_nodelet ${catkin_LIBRARIES} ${Snav_LIBRARY})

  install(TARGETS vio_odom_publisher so3cmd_to_qualcomm_nodelet trpycmd_to_snav_nodelet poscmd_to_snav_nodelet
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
  install(FILES nodelet_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

endif()
