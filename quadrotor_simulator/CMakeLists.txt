cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_simulator)

find_package(catkin REQUIRED COMPONENTS roscpp quadrotor_msgs nav_msgs sensor_msgs cmake_modules)
find_package(Eigen 3.0.0 REQUIRED)

include_directories(${EIGEN_INCLUDE_DIR})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
include_directories(include/odeint-v2)

catkin_package(
    DEPENDS eigen3
    CATKIN_DEPENDS roscpp quadrotor_msgs nav_msgs sensor_msgs
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

add_library(quadrotor_dynamics src/dynamics/Quadrotor.cpp)

add_executable(quadrotor_simulator_so3 src/quadrotor_simulator_so3.cpp)
add_executable(quadrotor_simulator_trpy src/quadrotor_simulator_trpy.cpp)

target_link_libraries(quadrotor_simulator_so3 quadrotor_dynamics)
target_link_libraries(quadrotor_simulator_trpy quadrotor_dynamics)

add_dependencies(quadrotor_dynamics quadrotor_msgs_generate_messages_cpp)
add_dependencies(quadrotor_simulator_so3 quadrotor_msgs_generate_messages_cpp)
add_dependencies(quadrotor_simulator_trpy quadrotor_msgs_generate_messages_cpp)

target_link_libraries(quadrotor_dynamics ${catkin_LIBRARIES} )
target_link_libraries(quadrotor_simulator_so3 ${catkin_LIBRARIES} )
target_link_libraries(quadrotor_simulator_trpy ${catkin_LIBRARIES} )

