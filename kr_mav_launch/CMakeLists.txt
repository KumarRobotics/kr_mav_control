cmake_minimum_required(VERSION 3.10)
project(kr_mav_launch)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
# install(DIRECTORY scripts/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
