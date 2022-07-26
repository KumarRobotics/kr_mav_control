kr_mav_control
=================

ROS packages for quadrotor control

[![Build Status](https://github.com/KumarRobotics/kr_mav_control/workflows/build/badge.svg)](https://github.com/KumarRobotics/kr_mav_control/actions?query=workflow%3Abuild)

### Stack includes:

  - `kr_mav_manager`: A manager for the kr_mav_control stack
  - `rqt_mav_manager`: GUI interface for common kr_mav_manager functions
  - `interfaces`: Translates `kr_mav_msgs/SO3Command` to corresponding robots interface.
  - `kr_quadrotor_simulator`: Simple quadrotor dynamics simulator
  - `kr_mav_msgs`: Common msgs used across packages
  - `kr_mav_controllers`: Position controllers
  - `trackers`: Different trackers under `kr_trackers`, and `kr_trackers_manager`

### Example use cases:

The multi robot examples uses following packages.

* [kr_multi_mav_manager](https://github.com/KumarRobotics/multi_mav_manager)
* [mrsl_quadrotor](https://github.com/KumarRobotics/mrsl_quadrotor)
* [waypoint_navigation_tool](https://github.com/KumarRobotics/waypoint_navigation_plugin)

[Running single robot with the included simple simulator](doc/QuadSim.md)

[Running multiple robots with the included simple simulator](doc/MultiSim.md)

[MATLAB interface with simple simulator](doc/MultiMatlab.md)

[Python interface with simple simulator](doc/PythonExample.md)

[Running with Gazebo](doc/QuadGazebo.md)

[Running multiple robots with Gazebo](doc/MultiGazebo.md)

This code has been tested with ROS Noetic on Ubuntu 20.04.

### Block Diagram

The following block diagram shows how the packages in the repo fit together.
![Block Diagram](doc/kr_mav_control_block_diag.png)

Further detailed breakdown of the blocks can be found in
the [PPT](doc/kr_mav_control_block_diagram.pptx)
