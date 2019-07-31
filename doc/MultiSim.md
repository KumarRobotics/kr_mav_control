## Example with multiple simulated robots

Clone and build [kr_ui](https://github.com/KumarRobotics/kr_ui) and [multi_mav_manager](https://github.com/KumarRobotics/multi_mav_manager) in your workspace

Helper bash scripts are added to launch multiple robots.
```
roscd quadrotor_simulator/scripts
./demo_sim.py 4
```
 * This will launch 4 robots in simulator
 * Switch to the `Kill` tab in the `tmux` window and press `Enter` to close everything
 * `dragonfly$ID` namespace is used for each robot. Each vehicle can be interfaced with the quadrotor_control topics/services in this namespace.
 * Use the rqt_multi_mav_manager GUI to control all the robots