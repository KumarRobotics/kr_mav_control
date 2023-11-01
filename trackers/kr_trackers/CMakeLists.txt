cmake_minimum_required(VERSION 3.5)

project(kr_trackers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(kr_trackers_msgs REQUIRED)
find_package(kr_trackers_manager REQUIRED)
find_package(kr_mav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

# include_directories(include)
ament_export_dependencies(kr_trackers
  rclcpp
  rclcpp_components
  kr_trackers_msgs
  kr_mav_msgs
  std_msgs
  kr_trackers_manager
  nav_msgs
  geometry_msgs
)

# target_link_libraries(kr_trackers
#   kr_trackers_manager)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(
  kr_trackers SHARED
  src/line_tracker_min_jerk_server_ros2.cpp
)
target_compile_definitions(kr_trackers
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(kr_trackers
  rclcpp
  rclcpp_components
  kr_trackers_msgs
  kr_mav_msgs
  std_msgs
  kr_trackers_manager
  nav_msgs
  geometry_msgs
)

# target_link_libraries(kr_trackers
# kr_trackers_manager
# )
  
rclcpp_components_register_nodes(kr_trackers "kr_trackers::LineTrackerMinJerk")
set(node_plugins "${node_plugins}kr_trackers::LineTrackerMinJerk;$<TARGET_FILE:kr_trackers>\n")
# target_link_libraries(kr_trackers kr_trackers_manager)
target_link_libraries(kr_trackers Eigen3::Eigen)

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

target_include_directories(kr_trackers
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS
  kr_trackers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
