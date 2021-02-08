cmake_minimum_required(VERSION 3.10)
project(kr_mav_manager)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             nav_msgs
             sensor_msgs
             geometry_msgs
             kr_mav_msgs
             kr_tracker_msgs
             std_msgs
             std_srvs
             message_generation
             actionlib)
find_package(Eigen3 REQUIRED NO_MODULE)

add_service_files(
  DIRECTORY
  srv
  FILES
  Vec4.srv
  GoalTimed.srv
  Circle.srv
  Lissajous.srv
  CompoundLissajous.srv)
generate_messages()

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  kr_mav_msgs
  kr_tracker_msgs
  message_runtime
  DEPENDS
  EIGEN3)

add_library(${PROJECT_NAME} src/manager.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} Eigen3::Eigen)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(mav_services src/mav_services.cpp)
target_link_libraries(mav_services PRIVATE ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} mav_services
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
