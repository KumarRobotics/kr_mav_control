cmake_minimum_required(VERSION 3.5)
project(kr_mavros_interface)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall)

# ROS 2 configuration
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kr_mav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# If mavros_msgs not found, don't build but do not cause a build error
if(NOT mavros_msgs_FOUND)
  message(WARNING "NOTE: mavros_msgs not found so not building kr_mavros_interface")
else()
  # Create shared library
  add_library(${PROJECT_NAME} SHARED src/so3cmd_to_mavros_nodelet.cpp)
  
  ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rclcpp_components
    nav_msgs
    geometry_msgs
    sensor_msgs
    kr_mav_msgs
    mavros_msgs
    tf2
    tf2_geometry_msgs
    Eigen3)

  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  # Register component
  rclcpp_components_register_nodes(${PROJECT_NAME} "SO3CmdToMavros")

  # Install
  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

  install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
  install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
endif()

ament_package()
