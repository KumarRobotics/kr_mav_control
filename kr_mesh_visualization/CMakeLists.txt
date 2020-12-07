cmake_minimum_required(VERSION 3.10)
project(kr_mesh_visualization)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall)

find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs topic_tools visualization_msgs)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  nav_msgs
  topic_tools
  visualization_msgs)

add_executable(${PROJECT_NAME} src/mesh_visualization.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${catkin_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY mesh/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mesh)
