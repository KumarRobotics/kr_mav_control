cmake_minimum_required(VERSION 2.8.3)
project(std_trackers)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs geometry_msgs tf trackers_manager)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_action_files(DIRECTORY action
  FILES
  LineTracker.action
  VelocityTracker.action
  CircleTracker.action
  TrajectoryTracker.action)
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS geometry_msgs tf trackers_manager
  DEPENDS EIGEN3)

add_library(initial_conditions src/initial_conditions.cpp)
target_link_libraries(initial_conditions ${catkin_LIBRARIES})

add_library(line_tracker_min_jerk_action src/line_tracker_min_jerk_server.cpp)
add_dependencies(line_tracker_min_jerk_action ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(line_tracker_min_jerk_action initial_conditions ${catkin_LIBRARIES})

#add_library(line_tracker_trapezoid src/line_tracker_trapezoid.cpp)
#add_dependencies(line_tracker_trapezoid ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
#target_link_libraries(line_tracker_trapezoid initial_conditions ${catkin_LIBRARIES})

add_library(line_tracker_distance_action src/line_tracker_distance_server.cpp)
add_dependencies(line_tracker_distance_action ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(line_tracker_distance_action initial_conditions ${catkin_LIBRARIES})

#add_library(line_tracker_yaw src/line_tracker_yaw.cpp)
#add_dependencies(line_tracker_yaw ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
#target_link_libraries(line_tracker_yaw initial_conditions ${catkin_LIBRARIES})

add_library(null_tracker src/null_tracker.cpp)
add_dependencies(null_tracker ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(null_tracker ${catkin_LIBRARIES})

add_library(velocity_tracker_action src/velocity_tracker_server.cpp)
add_dependencies(velocity_tracker_action ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(velocity_tracker_action ${catkin_LIBRARIES})

add_library(circle_tracker_action src/circle_tracker_server.cpp)
add_dependencies(circle_tracker_action ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(circle_tracker_action ${catkin_LIBRARIES})

add_library(smooth_vel_tracker_action src/smooth_vel_tracker_server.cpp)
add_dependencies(smooth_vel_tracker_action ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(smooth_vel_tracker_action initial_conditions ${catkin_LIBRARIES})

add_library(trajectory_tracker src/trajectory_tracker.cpp src/traj_gen.cpp)
add_dependencies(trajectory_tracker ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(trajectory_tracker initial_conditions ${catkin_LIBRARIES})

install(TARGETS
    initial_conditions
    line_tracker_min_jerk_action
    #line_tracker_trapezoid
    line_tracker_distance_action
    #line_tracker_yaw
    null_tracker
    velocity_tracker_action
    circle_tracker_action
    #smooth_vel_tracker
    trajectory_tracker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
