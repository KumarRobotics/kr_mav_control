## Example with multiple simulated robots and MATLAB (Compatible with 2018b and later versions)

## Running simple simulator with kr_python_interface

Helper bash scripts are added to launch multiple robots.
```
roscd kr_multi_mav_manager/scripts
./demo_sim.sh 1
```
 * This will launch 1 robots in simulator
 * Switch to the `Kill` tab in the `tmux` window and press `Enter` to close everything
 * `dragonfly$ID` namespace is used for each robot. Each vehicle can be interfaced with the kr_mav_control topics/services in this namespace.

In another terminal
```
rosrun kr_matlab_interface mav_example.py
```
