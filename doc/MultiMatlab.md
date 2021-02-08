## Example with multiple simulated robots and MATLAB (Compatible with 2018b and later versions)

MATLAB wrappers for `kr_mav_control` using the [Robotics Systems Toolbox](https://www.mathworks.com/help/robotics/index.html?s_tid=CRUX_lftnav) are available in `kr_matlab_interface`.

Make sure you have the toolbox installed. In addition, for the interface to work with `kr_mav_control`, ROS custom messages have to be generated.

In MATLAB console
```
roboticsAddons
```
 * Install
   * `Robotics System Toolbox Interface for ROS Custom Messages`
   * `Robotics System Toolbox UAV Library` (Only available for 2018b and later versions)

Once installed follow the following instructions to generate custom messages used in `kr_mav_control`. More info on generating ROS custom messages is [here](https://www.mathworks.com/help/robotics/ref/rosgenmsg.html)

```
cd ~/ws_ros/src/kr_mav_control
mkdir -p ~/matlab_msgs
cp -r kr_mav_msgs kr_mav_manager ~/matlab_msgs
cd ~/ws_ros/src/kr_mav_control/interfaces/kr_matlab_interface
cp kr_mav_msgs.patch.xml ~/matlab_msgs/kr_mav_msgs/package.xml
cp kr_mav_manager.patch.package.xml ~/matlab_msgs/kr_mav_manager/package.xml
cd ~/matlab_msgs
git clone https://github.com/ros/ros_comm_msgs.git
cd ros_comm_msgs
mv std_srvs ..
```
 * Newer format 2 of `package.xml` has issues with MATLAB message generation. Hence, an older patched version is used.
 * MATLABs inbuild `std_srvs` does not have all the needed services. Hence, we clone and compile it again locally.

Open MATLAB and run the following

```
rosgenmsg('~/matlab_msgs')
```

 * Follow the instructions spilled out in MATLAB console, i.e edit `javaclasspath.txt` and `addpath` with necessary locations.

Clone and build [kr_multi_mav_manager](https://github.com/KumarRobotics/multi_mav_manager) in your workspace

## Running simple simulator with kr_matlab_interface

Helper bash scripts are added to launch multiple robots.
```
roscd kr_multi_mav_manager/scripts
./demo_sim.sh 4
```
 * This will launch 4 robots in simulator
 * Switch to the `Kill` tab in the `tmux` window and press `Enter` to close everything
 * `dragonfly$ID` namespace is used for each robot. Each vehicle can be interfaced with the kr_mav_control topics/services in this namespace.

In MATLAB
```
cd ~/ws_ros/src/kr_mav_control/interfaces/kr_matlab_interface
example_interface('localhost', 4)
```
 * This starts the motors, calls takeoff and moves all the 4 MAVs with random velocity.
 * The first string is the hostname. Use `localhost` if the simulators are running on your own machine.
 * Copy and reuse the `example_interface.m` to develop/test your own awesome MATLAB based algorithms.
 * This is a minimal interface. More API specific to kr_mav_control can be added easily.
 * If due to some issue, MATLAB gets stuck in a loop, type `rosshutdown` at the console. This will shutdown all subscribers.

Happy `MATLABing`
