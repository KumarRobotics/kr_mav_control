cmake_minimum_required(VERSION 3.10)
project(kr_trackers)

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
  COMPONENTS actionlib
             std_msgs
             geometry_msgs
             nav_msgs
             tf
             kr_mav_msgs
             kr_tracker_msgs
             kr_trackers_manager)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  actionlib
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  kr_mav_msgs
  kr_tracker_msgs
  kr_trackers_manager
  DEPENDS
  EIGEN3)

add_library(
  ${PROJECT_NAME}
  src/initial_conditions.cpp
  src/circle_tracker_server.cpp
  src/initial_conditions.cpp
  src/line_tracker_distance_server.cpp
  src/line_tracker_min_jerk_server.cpp
  # src/line_tracker_trapezoid.cpp
  # src/line_tracker_yaw.cpp
  src/lissajous_adder_server.cpp
  src/lissajous_generator.cpp
  src/lissajous_tracker_server.cpp
  src/null_tracker.cpp
  src/smooth_vel_tracker_server.cpp
  src/trajectory_tracker.cpp
  src/traj_gen.cpp
  src/velocity_tracker.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} Eigen3::Eigen)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES nodelet_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(PROGRAMS
     scripts/twist_to_velocity_goal.py
     scripts/waypoints_to_action.py
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
