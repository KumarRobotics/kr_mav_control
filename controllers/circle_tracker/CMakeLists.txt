cmake_minimum_required(VERSION 2.8.3)
project(circle_tracker)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(catkin REQUIRED COMPONENTS controllers_manager cmake_modules std_msgs tf)
find_package(Eigen REQUIRED)

add_service_files(
  FILES DesVelAcc.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES std_msgs
)

catkin_package(
  LIBRARIES circle_tracker
  CATKIN_DEPENDS controllers_manager std_msgs
  DEPENDS Eigen
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_library(circle_tracker src/circle_tracker.cpp)
add_dependencies(circle_tracker ${catkin_EXPORTED_TARGETS})
target_link_libraries(circle_tracker ${catkin_LIBRARIES})

install(TARGETS circle_tracker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
