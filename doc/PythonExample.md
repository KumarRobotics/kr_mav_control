## Example with a simulated robot and Python

## Running simple simulator with kr_python_interface

```
roscd kr_multi_mav_manager/scripts
./demo_sim.sh 1
```
In another terminal
```
rosrun kr_python_interface mav_example.py
```
 * Switch to the `Kill` tab in the `tmux` window and press `Enter` to close everything
 * `dragonfly$ID` namespace is used for each robot. Each vehicle can be interfaced with the kr_mav_control topics/services in this namespace.