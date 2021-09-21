cmake_minimum_required(VERSION 3.10)
project(kr_quadrotor_simulator)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall)

find_package(catkin REQUIRED COMPONENTS geometry_msgs kr_mav_msgs nav_msgs roscpp sensor_msgs tf2_ros)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  kr_mav_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf2_ros
  DEPENDS
  EIGEN3
)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} INTERFACE ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(kr_quadrotor_dynamics src/dynamics/Quadrotor.cpp)
target_include_directories(kr_quadrotor_dynamics PUBLIC include)
target_link_libraries(kr_quadrotor_dynamics PUBLIC Eigen3::Eigen ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_so3 src/quadrotor_simulator_so3.cpp)
target_link_libraries(${PROJECT_NAME}_so3 PUBLIC kr_quadrotor_dynamics)
# add_dependencies(${PROJECT_NAME}_so3 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_trpy src/quadrotor_simulator_trpy.cpp)
target_link_libraries(${PROJECT_NAME}_trpy PUBLIC kr_quadrotor_dynamics)
# add_dependencies(${PROJECT_NAME}_trpy ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(
  TARGETS kr_quadrotor_dynamics ${PROJECT_NAME}_so3 ${PROJECT_NAME}_trpy
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
