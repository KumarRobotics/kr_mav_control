cmake_minimum_required(VERSION 2.8.3)
project(std_trackers)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(catkin REQUIRED COMPONENTS cmake_modules geometry_msgs tf trackers_manager)
find_package(Eigen REQUIRED)

include_directories(include ${EIGEN_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS geometry_msgs tf trackers_manager
  DEPENDS Eigen)

add_library(initial_conditions src/initial_conditions.cpp)

add_library(line_tracker_min_jerk src/line_tracker_min_jerk.cpp)
add_dependencies(line_tracker_min_jerk ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(line_tracker_min_jerk initial_conditions ${catkin_LIBRARIES})

add_library(line_tracker_trapezoid src/line_tracker_trapezoid.cpp)
add_dependencies(line_tracker_trapezoid ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(line_tracker_trapezoid ${catkin_LIBRARIES})

add_library(line_tracker_distance src/line_tracker_distance.cpp)
add_dependencies(line_tracker_distance ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(line_tracker_distance ${catkin_LIBRARIES})

add_library(line_tracker_yaw src/line_tracker_yaw.cpp)
add_dependencies(line_tracker_yaw ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(line_tracker_yaw ${catkin_LIBRARIES})

add_library(null_tracker src/null_tracker.cpp)
add_dependencies(null_tracker ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(null_tracker ${catkin_LIBRARIES})

add_library(velocity_tracker src/velocity_tracker.cpp)
add_dependencies(velocity_tracker ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(velocity_tracker ${catkin_LIBRARIES})

install(TARGETS
    initial_conditions
    line_tracker_min_jerk
    line_tracker_trapezoid
    line_tracker_distance
    line_tracker_yaw
    null_tracker
    velocity_tracker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
