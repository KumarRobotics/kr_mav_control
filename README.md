quadrotor_control
=================

ROS packages for quadrotor control

**Note:** There are packages that require submodules, you need to initialize the submodules by running the following in the repository directory:
```bash
git submodule init
git submodule update
```

### Block Diagram

The following block diagram shows how the packages in the repo fit together.
![Block Diagram](doc/quad_control_block_diag.png)

### Stacks include:
  - `mav_manager`: A manager for the quadrotor_control stack
  - `rqt_mav_manager`: GUI interface for common mav_manager functions
  - `xyz_interface`: Translates `quadrotor_msgs/SO3Command` to corresponding `xyz` robots interface.
  - `quadrotor_simulator`: Simple quadrotor dynamics simulator
  - `quadrotor_msgs`: Common msgs used accross packages
  - `so3_control`: The main controller
  - `trackers`: Different trackers under `std_trackers`, and `trackers_manager`

### Example use cases

[Running single robot with the included simple simulator](doc/QuadSim.md)

[Running multiple robots with the included simple simulator](doc/MultiSim.md)

[Running with Gazebo](doc/QuadGazebo.md)
