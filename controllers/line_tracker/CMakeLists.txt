cmake_minimum_required(VERSION 2.8.3)
project(line_tracker)

find_package(catkin REQUIRED COMPONENTS roscpp quadrotor_msgs geometry_msgs nav_msgs tf nodelet controllers_manager)
find_package(Eigen REQUIRED)

include_directories(${EIGEN_INCLUDE_DIR})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_service_files(
  FILES
  DesVelAcc.srv
)
generate_messages()

catkin_package(
    DEPENDS roscpp quadrotor_msgs geometry_msgs nav_msgs tf nodelet controllers_manager
    CATKIN_DEPENDS # TODO
    LIBRARIES # TODO
)

add_library(line_tracker_min_jerk src/line_tracker_min_jerk.cpp)
add_library(line_tracker_trapezoid src/line_tracker_trapezoid.cpp)
add_library(line_tracker_distance src/line_tracker_distance.cpp)

add_dependencies(line_tracker_min_jerk quadrotor_msgs_generate_messages_cpp)
add_dependencies(line_tracker_trapezoid quadrotor_msgs_generate_messages_cpp)
add_dependencies(line_tracker_distance quadrotor_msgs_generate_messages_cpp)

target_link_libraries(line_tracker_min_jerk ${catkin_LIBRARIES} )
target_link_libraries(line_tracker_trapezoid ${catkin_LIBRARIES} )
target_link_libraries(line_tracker_distance ${catkin_LIBRARIES} )