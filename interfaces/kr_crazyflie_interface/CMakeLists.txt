cmake_minimum_required(VERSION 3.10)
project(kr_crazyflie_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kr_mav_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED src/so3cmd_to_crazyflie_component.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_components kr_mav_msgs nav_msgs geometry_msgs)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
rclcpp_components_register_nodes(${PROJECT_NAME} "SO3CmdToCrazyflie")

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
