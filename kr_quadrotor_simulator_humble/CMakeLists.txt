cmake_minimum_required(VERSION 3.8)
project(kr_quadrotor_simulator_humble)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kr_mav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include/)
set(HEADER_FILES include/kr_quadrotor_simulator_humble/Quadrotor.hpp)
add_library(kr_quadrotor_dynamics src/dynamics/Quadrotor.cpp ${HEADER_FILES})
ament_export_targets(kr_quadrotor_dynamics HAS_LIBRARY_TARGET)
target_link_libraries(kr_quadrotor_dynamics Eigen3::Eigen)
ament_export_dependencies(kr_quadrotor_dynamics rclcpp rclcpp_action nav_msgs std_msgs sensor_msgs kr_mav_msgs kr_trackers_msgs Eigen3::Eigen)

target_include_directories(kr_quadrotor_dynamics
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


install(
  TARGETS kr_quadrotor_dynamics
  EXPORT kr_quadrotor_dynamics
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_executable(${PROJECT_NAME}_so3 src/humble_quadrotor_simulator_so3.cpp)
ament_target_dependencies(${PROJECT_NAME}_so3 rclcpp kr_mav_msgs geometry_msgs nav_msgs sensor_msgs tf2)
target_link_libraries( ${PROJECT_NAME}_so3 Eigen3::Eigen)
target_link_libraries( ${PROJECT_NAME}_so3 kr_quadrotor_dynamics)

install(TARGETS
  ${PROJECT_NAME}_so3
  DESTINATION lib/${PROJECT_NAME}/
)
# target_include_directories( ${PROJECT_NAME}_so3
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
