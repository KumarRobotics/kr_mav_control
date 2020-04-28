cmake_minimum_required(VERSION 2.8.3)
project(qc_serial_interface)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

find_package(catkin REQUIRED COMPONENTS roscpp quadrotor_msgs sensor_msgs nodelet)
find_package(Boost REQUIRED COMPONENTS thread system)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp quadrotor_msgs sensor_msgs nodelet
  DEPENDS)

add_library(asio_serial_device src/ASIOSerialDevice.cc)
target_link_libraries(asio_serial_device ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})

add_library(encode_msgs src/encode_msgs.cpp)
add_dependencies(encode_msgs ${catkin_EXPORTED_TARGETS})
target_link_libraries(encode_msgs ${catkin_LIBRARIES})

add_library(quad_encode_msg_nodelet src/quad_encode_msg_nodelet.cpp)
add_dependencies(quad_encode_msg_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(quad_encode_msg_nodelet encode_msgs ${catkin_LIBRARIES})

add_library(decode_msgs src/decode_msgs.cpp)
add_dependencies(decode_msgs ${catkin_EXPORTED_TARGETS})
target_link_libraries(decode_msgs ${catkin_LIBRARIES})

add_library(quad_decode_msg_nodelet src/quad_decode_msg_nodelet.cpp)
add_dependencies(quad_decode_msg_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(quad_decode_msg_nodelet decode_msgs ${catkin_LIBRARIES})

add_library(quad_serial_comm_nodelet src/quad_serial_comm_nodelet.cpp src/asctec_serial_interface.cpp)
add_dependencies(quad_serial_comm_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(quad_serial_comm_nodelet asio_serial_device ${catkin_LIBRARIES})

install(TARGETS
    asio_serial_device encode_msgs quad_encode_msg_nodelet decode_msgs quad_decode_msg_nodelet quad_serial_comm_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES nodelet_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
