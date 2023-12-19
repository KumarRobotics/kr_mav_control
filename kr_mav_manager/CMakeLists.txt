cmake_minimum_required(VERSION 3.5)
project(kr_mav_manager)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kr_mav_msgs REQUIRED)
find_package(kr_trackers_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
	"srv/Circle.srv"
	"srv/CompoundLissajous.srv"
	"srv/GoalTimed.srv"
	"srv/Lissajous.srv"
	"srv/Vec4.srv"
)

## Create library

include_directories(include/)

add_library(manager src/manager.cpp)

ament_export_targets(manager HAS_LIBRARY_TARGET)
ament_target_dependencies(manager rclcpp rclcpp_action nav_msgs std_msgs sensor_msgs kr_trackers_msgs kr_mav_msgs)
target_link_libraries(manager Eigen3::Eigen)
ament_export_dependencies(manager rclcpp rclcpp_action nav_msgs std_msgs sensor_msgs kr_mav_msgs kr_trackers_msgs Eigen3::Eigen)

install(
  TARGETS manager
  EXPORT manager
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

##


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
