# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_simulator)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp quadrotor_msgs nav_msgs sensor_msgs)
find_package(Eigen3 REQUIRED)
#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
include_directories(include/odeint-v2)

# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()



#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#find_package(Boost REQUIRED COMPONENTS thread)
#include_directories(${Boost_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
#add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})



# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
add_library(quadrotor_dynamics src/dynamics/Quadrotor.cpp)

# CATKIN_MIGRATION
# use link_directories() include_directories(), add_definitions(), target_link_libraries() or set_target_properties
# rosbuild_add_link_flags(quadrotor_dynamics -Wl,--as-needed)

add_executable(quadrotor_simulator_so3 src/quadrotor_simulator_so3.cpp)
target_link_libraries(quadrotor_simulator_so3 quadrotor_dynamics)

# CATKIN_MIGRATION
# use link_directories() include_directories(), add_definitions(), target_link_libraries() or set_target_properties
# rosbuild_add_link_flags(quadrotor_simulator_so3 -Wl,--as-needed)

add_executable(quadrotor_simulator_trpy src/quadrotor_simulator_trpy.cpp)
target_link_libraries(quadrotor_simulator_trpy quadrotor_dynamics)

add_dependencies(quadrotor_dynamics quadrotor_msgs_generate_messages_cpp)
add_dependencies(quadrotor_simulator_so3 quadrotor_msgs_generate_messages_cpp)
add_dependencies(quadrotor_simulator_trpy quadrotor_msgs_generate_messages_cpp)

target_link_libraries(quadrotor_dynamics ${catkin_LIBRARIES} )
target_link_libraries(quadrotor_simulator_so3 ${catkin_LIBRARIES} )
target_link_libraries(quadrotor_simulator_trpy ${catkin_LIBRARIES} )


# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roscpp quadrotor_msgs nav_msgs sensor_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)