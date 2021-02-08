cmake_minimum_required(VERSION 3.10)
project(kr_rosflight_interface)

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

# If rosflight_msgs not found, don't build but do not cause a build error
find_package(rosflight_msgs)

if(NOT ${rosflight_msgs_FOUND})
  message(WARNING "NOTE: rosflight_msgs not found so not building kr_rosflight_interface")
else()
  catkin_package(
    LIBRARIES
    ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    nav_msgs
    geometry_msgs
    kr_mav_msgs
    rosflight_msgs
    nodelet
    DEPENDS
    EIGEN3)

  add_library(${PROJECT_NAME} src/so3cmd_to_rosflight_nodelet.cpp)
  target_include_directories(${PROJECT_NAME} PUBLIC ${caktin_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
                                                    ${rosflight_msgs_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} Eigen3::Eigen)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

  install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  install(FILES nodelet_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif()
