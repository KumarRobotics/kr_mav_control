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
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(kr_trackers_msgs REQUIRED)
find_package(kr_mav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(trackers_manager SHARED
  src/trackers_manager_ros2.cpp)
target_compile_definitions(trackers_manager
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(trackers_manager
  "rclcpp"
  "rclcpp_components"
  "kr_trackers_msgs"
  "kr_mav_msgs"
  "pluginlib"
  "nav_msgs")

rclcpp_components_register_nodes(trackers_manager "kr_trackers_manager::TrackersManager")
set(node_plugins "${node_plugins}kr_trackers_manager::TrackersManager;$<TARGET_FILE:component>\n")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()


install(TARGETS
  trackers_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# 
#   find_package(ament_cmake_pytest REQUIRED)
#   find_package(launch_testing_ament_cmake REQUIRED)
#   find_package(rmw_implementation_cmake REQUIRED)
# 
#   file(GENERATE
#     OUTPUT
#     "${CMAKE_CURRENT_BINARY_DIR}/test_ament_index/$<CONFIG>/share/ament_index/resource_index/node_plugin/${PROJECT_NAME}"
#     CONTENT "${node_plugins}")
# 
#   set(generated_python_files)
#   macro(tests)
#     set(MANUAL_COMPOSITION_EXECUTABLE $<TARGET_FILE:manual_composition>)
#     set(LINKTIME_COMPOSITION_EXECUTABLE $<TARGET_FILE:linktime_composition>)
#     set(DLOPEN_COMPOSITION_EXECUTABLE $<TARGET_FILE:dlopen_composition>)
#     set(TALKER_LIBRARY $<TARGET_FILE:talker_component>)
#     set(LISTENER_LIBRARY $<TARGET_FILE:listener_component>)
#     set(SERVER_LIBRARY $<TARGET_FILE:server_component>)
#     set(CLIENT_LIBRARY $<TARGET_FILE:client_component>)
#     set(NODE_LIKE_LISTENER_LIBRARY $<TARGET_FILE:node_like_listener_component>)
#     set(EXPECTED_OUTPUT_ALL "${CMAKE_CURRENT_SOURCE_DIR}/test/composition_all")
#     set(EXPECTED_OUTPUT_PUBSUB "${CMAKE_CURRENT_SOURCE_DIR}/test/composition_pubsub")
#     set(EXPECTED_OUTPUT_SRV "${CMAKE_CURRENT_SOURCE_DIR}/test/composition_srv")
# 
#     set(test_names
#       "test_dlopen_composition"
#       "test_linktime_composition"
#       "test_manual_composition"
#     )
#     foreach(test_name IN LISTS test_names)
#       configure_file(
#         test/${test_name}.py.in
#         ${test_name}${target_suffix}.py.genexp
#         @ONLY
#       )
#       file(GENERATE
#         OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py"
#         INPUT "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}.py.genexp"
#       )
#       add_launch_test(
#         "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py"
#         TARGET ${test_name}${target_suffix}
#         ENV RMW_IMPLEMENTATION=${rmw_implementation}
#         APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_CURRENT_BINARY_DIR}/test_ament_index/$<CONFIG>
#         APPEND_LIBRARY_DIRS "${append_library_dirs}"
#         TIMEOUT 60
#       )
#       list(
#         APPEND generated_python_files
#       "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py")
#     endforeach()
#   endmacro()
# 
#   set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}")
#   if(WIN32)
#     set(append_library_dirs "${append_library_dirs}/$<CONFIG>")
#   endif()
# 
#   call_for_each_rmw_implementation(tests)
# 
#   find_package(ament_cmake_flake8 REQUIRED)
#   ament_flake8(
#     TESTNAME "flake8_generated_launch"
#     # the generated code might contain longer lines for templated types
#     MAX_LINE_LENGTH 999
#     ${generated_python_files})
# endif()
# 
# # Install launch files.
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()

