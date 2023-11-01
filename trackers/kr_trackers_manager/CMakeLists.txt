cmake_minimum_required(VERSION 3.5)

project(kr_trackers_manager)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(kr_trackers_msgs REQUIRED)
find_package(kr_mav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
# find_package(nav_msgs REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)
# ament_export_include_directories(include/${PROJECT_NAME})

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(kr_trackers_manager SHARED
  src/trackers_manager_ros2.cpp)
target_compile_definitions(kr_trackers_manager
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(kr_trackers_manager
  rclcpp
  rclcpp_components
  rcutils
  kr_trackers_msgs
  kr_mav_msgs
  pluginlib
  )

rclcpp_components_register_nodes(kr_trackers_manager "kr_trackers_manager::TrackersManager")
set(node_plugins "${node_plugins}kr_trackers_manager::TrackersManager;$<TARGET_FILE:component>\n")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

target_include_directories(kr_trackers_manager
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# ament_export_dependencies()

install(
  TARGETS kr_trackers_manager
  EXPORT kr_trackers_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  DIRECTORY include/
  DESTINATION include
)

#ament_export_targets(export_kr_trackers_manager HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(kr_trackers_manager)
ament_export_dependencies(
    ament_cmake
    rclcpp
    pluginlib
    rclcpp_components
    rcutils
    kr_trackers_msgs
    kr_mav_msgs
    pluginlib
)

ament_package()
