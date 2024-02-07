cmake_minimum_required(VERSION 3.8)
project(kr_quadrotor_simulator_humble)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED COMPONENTS geometry_msgs kr_mav_msgs nav_msgs roscpp sensor_msgs tf2) 
find_package(Eigen3 REQUIRED)


include_directories(${CMAKE_SOURCE_DIR}/include)
install(
  DIRECTORY include/
  DESTINATION include
)

add_library(kr_quadrotor_dynamics src/dynamics/Quadrotor.cpp)
target_include_directories(kr_quadrotor_dynamics PUBLIC include)
target_link_libraries(kr_quadrotor_dynamics PUBLIC Eigen3::Eigen ${PROJECT_NAME})


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
