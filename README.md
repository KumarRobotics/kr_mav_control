quadrotor_control
=================

ROS packages for quadrotor control

**Note:** There are packages that require submodules, you need to initialize the submodules by running the following in the repository directory:
```bash
git submodule init
git submodule update
```

### Stacks include:
  - `kr_mav_manager`: A manager for the quadrotor_control stack
  - `rqt_kr_mav_manager`: GUI interface for common kr_mav_manager functions
  - `xyz_interface`: Translates `kr_quadrotor_msgs/SO3Command` to corresponding `xyz` robots interface.
  - `kr_quadrotor_simulator`: Simple quadrotor dynamics simulator
  - `kr_quadrotor_msgs`: Common msgs used accross packages
  - `kr_so3_control`: The main controller
  - `trackers`: Different trackers under `kr_std_trackers`, and `kr_trackers_manager`

### Example use cases

[Running single robot with the included simple simulator](doc/QuadSim.md)

[Running multiple robots with the included simple simulator](doc/MultiSim.md)

[MATLAB interface with simple simulator](doc/MultiMatlab.md)

[Running with Gazebo](doc/QuadGazebo.md)

[Running multiple robots with Gazebo](doc/MultiGazebo.md)

### Block Diagram

The following block diagram shows how the packages in the repo fit together.
![Block Diagram](doc/quad_control_block_diag.png)

Further detailed breakdown of the blocks can be found in
the [PPT](doc/quadrotor_control_block_diagram.pptx)