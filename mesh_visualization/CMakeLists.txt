cmake_minimum_required(VERSION 2.8.3)
project(mesh_visualization)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs visualization_msgs)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(${Boost_INCLUDE_DIRS})

catkin_package(
    DEPENDS roscpp nav_msgs visualization_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
add_executable(mesh_visualization src/mesh_visualization.cpp)

target_link_libraries(mesh_visualization  ${catkin_LIBRARIES})