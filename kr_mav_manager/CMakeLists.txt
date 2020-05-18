cmake_minimum_required(VERSION 2.8.12)
project(kr_mav_manager)

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
  kr_mav_msgs
  kr_tracker_msgs
  std_msgs
  std_srvs
  message_generation
  actionlib)

find_package(Eigen3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_service_files(
  DIRECTORY srv
  FILES
    Vec4.srv
    GoalTimed.srv
    Circle.srv
    Lissajous.srv
    CompoundLissajous.srv
)
generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kr_mav_manager
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    sensor_msgs
    geometry_msgs
    kr_mav_msgs
    kr_tracker_msgs
    std_msgs
    message_runtime
  DEPENDS EIGEN3)

add_library(kr_mav_manager src/manager.cpp)
add_dependencies(kr_mav_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(kr_mav_manager ${catkin_LIBRARIES})

add_executable(mav_services src/mav_services.cpp)
target_link_libraries(mav_services kr_mav_manager ${catkin_LIBRARIES})

install(TARGETS
    kr_mav_manager
    mav_services
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
