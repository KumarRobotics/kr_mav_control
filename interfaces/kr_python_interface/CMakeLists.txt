cmake_minimum_required(VERSION 2.8.3)
project(kr_python_interface)

find_package(catkin REQUIRED)
catkin_python_setup()

catkin_package()

install(PROGRAMS scripts/kr_python_interface
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
