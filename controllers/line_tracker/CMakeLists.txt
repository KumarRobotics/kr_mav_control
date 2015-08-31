cmake_minimum_required(VERSION 2.8.3)
project(line_tracker)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS geometry_msgs tf controllers_manager cmake_modules)
find_package(Eigen REQUIRED)

include_directories(include ${EIGEN_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_service_files(DIRECTORY srv
  FILES
  DesVelAcc.srv)
generate_messages()

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS geometry_msgs tf controllers_manager
  DEPENDS Eigen)

add_library(line_tracker_min_jerk src/line_tracker_min_jerk.cpp)
add_dependencies(line_tracker_min_jerk ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(line_tracker_min_jerk ${catkin_LIBRARIES})

add_library(line_tracker_trapezoid src/line_tracker_trapezoid.cpp)
add_dependencies(line_tracker_trapezoid ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(line_tracker_trapezoid ${catkin_LIBRARIES})

add_library(line_tracker_distance src/line_tracker_distance.cpp)
add_dependencies(line_tracker_distance ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(line_tracker_distance ${catkin_LIBRARIES})

add_library(line_tracker_yaw src/line_tracker_yaw.cpp)
add_dependencies(line_tracker_yaw ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(line_tracker_yaw ${catkin_LIBRARIES})
