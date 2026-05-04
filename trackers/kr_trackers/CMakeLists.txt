cmake_minimum_required(VERSION 3.10)
project(kr_trackers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(kr_mav_msgs REQUIRED)
find_package(kr_tracker_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
  include
  utils
)

set(plugins_dependencies
  pluginlib
)

set(node_dependencies
  ${plugins_dependencies}
  rclcpp
  rcutils
  rclcpp_components
  rclcpp_action
  rclcpp_lifecycle
  kr_mav_msgs
  kr_tracker_msgs
  nav_msgs
  tf2_geometry_msgs
  tf2
)

add_library(${PROJECT_NAME}_plugins SHARED 
  src/line_tracker_distance_server.cpp
  src/line_tracker_min_jerk_server.cpp 
  src/null_tracker.cpp
  src/poly_tracker.cpp
  src/velocity_tracker.cpp
  src/initial_conditions.cpp
  src/circle_tracker_server.cpp
  src/lissajous_generator.cpp
  src/lissajous_tracker_server.cpp
  src/lissajous_adder_server.cpp)
ament_target_dependencies(${PROJECT_NAME}_plugins ${node_dependencies})


install(TARGETS 
  ${PROJECT_NAME}_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES plugins_description.xml
  DESTINATION share
)


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_plugins)
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins_description.xml)
ament_package()
