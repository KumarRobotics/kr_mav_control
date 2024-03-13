cmake_minimum_required(VERSION 3.10)
project(kr_mav_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(kr_mav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_library(kr_mav_pid_control SHARED src/pid_control_component.cpp src/PIDControl.cpp)
ament_target_dependencies(kr_mav_pid_control rclcpp rclcpp_components kr_mav_msgs nav_msgs std_msgs tf2 tf2_geometry_msgs)
target_link_libraries(kr_mav_pid_control Eigen3::Eigen)
rclcpp_components_register_nodes(kr_mav_pid_control "PIDControlComponent")

add_library(kr_mav_so3_control SHARED src/so3_control_component.cpp src/SO3Control.cpp src/so3_trpy_control.cpp)
ament_target_dependencies(kr_mav_so3_control rclcpp rclcpp_components kr_mav_msgs nav_msgs std_msgs geometry_msgs tf2 tf2_geometry_msgs)
target_link_libraries(kr_mav_so3_control Eigen3::Eigen)
rclcpp_components_register_nodes(kr_mav_so3_control "SO3ControlComponent")
rclcpp_components_register_nodes(kr_mav_so3_control "SO3TRPYControlComponent")

install(TARGETS
  kr_mav_pid_control
  kr_mav_so3_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  include_directories(include)
  ament_add_gtest_executable(so3_control_component_test test/so3_control_component_test.cpp)
  ament_target_dependencies(so3_control_component_test
    std_msgs
    kr_mav_msgs
    nav_msgs
    rclcpp
  )

  add_launch_test(test/launch/so3_control_component_test.test.py TIMEOUT 80 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

endif()

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
