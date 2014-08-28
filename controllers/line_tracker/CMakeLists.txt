cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

rosbuild_add_library(line_tracker_min_jerk src/line_tracker_min_jerk.cpp)
rosbuild_add_link_flags(line_tracker_min_jerk -Wl,--as-needed)

rosbuild_add_library(line_tracker_trapezoid src/line_tracker_trapezoid.cpp)
rosbuild_add_link_flags(line_tracker_trapezoid -Wl,--as-needed)

rosbuild_add_library(line_tracker_distance src/line_tracker_distance.cpp)
rosbuild_add_link_flags(line_tracker_distance -Wl,--as-needed)

rosbuild_add_library(line_tracker_yaw src/line_tracker_yaw.cpp)
rosbuild_add_link_flags(line_tracker_yaw -Wl,--as-needed)
