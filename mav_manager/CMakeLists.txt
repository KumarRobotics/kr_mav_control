cmake_minimum_required(VERSION 2.8.12)
project(mav_manager)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  quadrotor_msgs
  trackers_manager
  std_trackers
  std_msgs
  cmake_modules
  message_generation)

find_package(Eigen REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR})

add_service_files(
  DIRECTORY srv
  FILES
    Bool.srv
    Empty.srv
    Vec4.srv
)
generate_messages()

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    sensor_msgs
    geometry_msgs
    quadrotor_msgs
    trackers_manager
    std_trackers
    std_msgs
  DEPENDS Eigen)

add_executable(mav_manager src/manager.cpp src/sample.cpp)
add_dependencies(mav_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(mav_manager ${catkin_LIBRARIES})

add_executable(mav_manager_sub src/manager.cpp src/sample_sub.cpp)
add_dependencies(mav_manager_sub ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(mav_manager_sub ${catkin_LIBRARIES})
