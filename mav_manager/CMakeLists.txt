cmake_minimum_required(VERSION 2.8.12)
project(mav_manager)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  quadrotor_msgs
  trackers_manager
  std_trackers
  std_msgs
  std_srvs
  cmake_modules
  message_generation)

find_package(Eigen REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR})

add_service_files(
  DIRECTORY srv
  FILES
    Vec4.srv
    GoalTimed.srv
    Circle.srv
)
generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mav_manager
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    sensor_msgs
    geometry_msgs
    quadrotor_msgs
    trackers_manager
    std_trackers
    std_msgs
    message_runtime
  DEPENDS Eigen)

add_library(mav_manager src/manager.cpp)
add_dependencies(mav_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(mav_manager ${catkin_LIBRARIES})

add_executable(mav_services src/sample.cpp)
target_link_libraries(mav_services mav_manager ${catkin_LIBRARIES})

install(TARGETS
    mav_manager
    mav_services
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
