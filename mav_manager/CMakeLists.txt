cmake_minimum_required(VERSION 2.8.12)
project(mav_manager)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  quadrotor_msgs
  trackers_manager
  std_trackers
  trajectory_trackers
  std_msgs
  std_srvs
  cmake_modules
  message_generation)

find_package(Eigen REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR})

add_service_files(
  DIRECTORY srv
  FILES
    Bool.srv
    Trigger.srv
    Vec4.srv
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
    trajectory_trackers
    std_msgs
    message_runtime
  DEPENDS Eigen)

add_library(mav_manager src/manager.cpp)
add_dependencies(mav_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(mav_manager ${catkin_LIBRARIES})


add_executable(mav_manager_node src/sample.cpp)
target_link_libraries(mav_manager_node mav_manager ${catkin_LIBRARIES})

add_executable(mav_manager_sub src/sample_sub.cpp)
target_link_libraries(mav_manager_sub mav_manager ${catkin_LIBRARIES})

add_executable(mav_manager_services src/sample_service_server.cpp)
target_link_libraries(mav_manager_services mav_manager ${catkin_LIBRARIES})
