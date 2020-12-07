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

find_package(catkin REQUIRED COMPONENTS nav_msgs kr_mav_msgs sensor_msgs tf2_ros)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  tf2_ros
  nav_msgs
  sensor_msgs
  kr_mav_msgs
  DEPENDS
  EIGEN3)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} INTERFACE ${catkin_LIBRARIES})

add_library(kr_quadrotor_dynamics src/dynamics/Quadrotor.cpp)
target_include_directories(kr_quadrotor_dynamics PUBLIC include)
target_link_libraries(kr_quadrotor_dynamics PUBLIC Eigen3::Eigen)

add_executable(kr_quadrotor_simulator_so3 src/quadrotor_simulator_so3.cpp)
target_link_libraries(kr_quadrotor_simulator_so3 PUBLIC kr_quadrotor_dynamics ${PROJECT_NAME})
# add_dependencies(kr_quadrotor_simulator_so3 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(kr_quadrotor_simulator_trpy src/quadrotor_simulator_trpy.cpp)
target_link_libraries(kr_quadrotor_simulator_trpy PUBLIC kr_quadrotor_dynamics ${PROJECT_NAME})
# add_dependencies(kr_quadrotor_simulator_trpy ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(
  TARGETS kr_quadrotor_dynamics kr_quadrotor_simulator_so3 kr_quadrotor_simulator_trpy
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
