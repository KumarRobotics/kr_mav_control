cmake_minimum_required(VERSION 2.8.3)
project(kr_python_interface)

find_package(catkin REQUIRED)
catkin_python_setup()

catkin_package()

install(PROGRAMS src/kr_python_interface/mav_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
