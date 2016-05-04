cmake_minimum_required(VERSION 2.8.3)
project(kquad_interface)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp nodelet quadrotor_msgs sensor_msgs geometry_msgs)
find_package(Threads REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp nodelet quadrotor_msgs sensor_msgs geometry_msgs
  DEPENDS)

add_library(kquad_interface src/kQuadInterface.cc
  src/SerialDevice.cc
  src/kBotPacket2.c
  src/crc32.c
  src/kBotPacketParser.cc
  src/kQuadInterfaceDataTypes.cc)
target_link_libraries(kquad_interface ${CMAKE_THREAD_LIBS_INIT})

add_library(kquad_interface_nodelet src/kquad_interface_nodelet.cpp)
target_link_libraries(kquad_interface_nodelet kquad_interface ${catkin_LIBRARIES})

install(TARGETS
    kquad_interface
    kquad_interface_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(FILES nodelet_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
