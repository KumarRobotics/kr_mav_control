## Example with multiple simulated robots

Matlab wrappers for quadrotor_control using [Robotics Systems Toolbox](https://www.mathworks.com/help/robotics/index.html?s_tid=CRUX_lftnav) are available in `matlab_interface`.

Make sure you have to toolbox installed. In addition, for the interface to work custom messages have to be generated for MATLAB.

Install the Robotics System Toolbox Interface for ROS Custom Messages add-on using `roboticsAddons` to use this function in MATLAB console.
Once installed follow the following instructions to generate custom messages used in `quadrotor_control`. More info [at](https://www.mathworks.com/help/robotics/ref/rosgenmsg.html)

```
cd ~/ws_ros/src/quadrotor_control
mkdir -p ~/matlab_msgs
cp -r quadrotor_msgs mav_manager ~/matlab_msgs
cd ~/ws_ros/src/quadrotor_control/matlab_interface
cp quadrotor_msgs.patch.package.xml ~/matlab_msgs/quadrotor_msgs/package.xml
cd ~/matlab_msgs
git clone https://github.com/ros/ros_comm_msgs.git
cd ros_comm_msgs
mv std_srvs ..
```
 * Newer format=2 package.xml has issues with MATLAB message generation. Hence an older patched version is used.
 * MATLABs inbuild `std_srvs` does not have all the needed services. Hence we compile it again locally.
Open MATLAB and run the following

```
rosgenmsg('~/matlab_msgs')
```

 * Follow the instructions spilled out in MATLAB console to use the custom_msgs in MATLAB, i.e edit `javaclasspath.txt` and `addpath`


Clone and build [kr_ui](https://github.com/KumarRobotics/kr_ui) and [multi_mav_manager](https://github.com/KumarRobotics/multi_mav_manager) in your workspace

Helper bash scripts are added to launch multiple robots.
```
roscd quadrotor_simulator/scripts
./demo_sim.py 4
```
 * This will launch 4 robots in simulator
 * Switch to the `Kill` tab in the `tmux` window and press `Enter` to close everything
 * `dragonfly$ID` namespace is used for each robot. Each vehicle can be interfaced with the quadrotor_control topics/services in this namespace.

In MATLAB
```
cd ~/ws_ros/src/quadrotor_control/matlab_interface
example_interface(4)
```
 * This starts the motors, calls takeoff and moves the 4 MAVs with random velocity.
 * Copy and reuse the `example_interface.m` to develop/test your own awesome MATLAB based algorithms.
 * This is a minimal interface. More API specific to quadrotor_control can be added easily.

Happy `MATLABing`