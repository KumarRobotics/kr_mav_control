cmake_minimum_required(VERSION 2.8.3)
project(std_trackers)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(catkin REQUIRED COMPONENTS cmake_modules geometry_msgs tf trackers_manager dynamic_reconfigure)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(cfg/Gains.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES flat_state
  CATKIN_DEPENDS geometry_msgs tf trackers_manager
  DEPENDS Eigen)

add_library(flat_state src/flat_state.cpp)
target_link_libraries(flat_state ${catkin_LIBRARIES})

add_library(line_tracker_min_jerk src/line_tracker_min_jerk.cpp)
add_dependencies(line_tracker_min_jerk ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(line_tracker_min_jerk flat_state ${catkin_LIBRARIES})

add_library(line_tracker_trapezoid src/line_tracker_trapezoid.cpp)
add_dependencies(line_tracker_trapezoid ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(line_tracker_trapezoid flat_state ${catkin_LIBRARIES})

add_library(line_tracker_distance src/line_tracker_distance.cpp)
add_dependencies(line_tracker_distance ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(line_tracker_distance flat_state ${catkin_LIBRARIES})

add_library(line_tracker_yaw src/line_tracker_yaw.cpp)
add_dependencies(line_tracker_yaw ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(line_tracker_yaw flat_state ${catkin_LIBRARIES})

add_library(null_tracker src/null_tracker.cpp)
add_dependencies(null_tracker ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(null_tracker ${catkin_LIBRARIES})

add_library(velocity_tracker src/velocity_tracker.cpp)
add_dependencies(velocity_tracker ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(velocity_tracker ${catkin_LIBRARIES})

install(TARGETS
    flat_state
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

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
