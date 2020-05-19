## Example with multiple simulated robots

Clone and build [kr_ui](https://github.com/KumarRobotics/kr_ui) and [multi_mav_manager](https://github.com/KumarRobotics/multi_mav_manager) in your workspace

Helper bash scripts are added to launch multiple robots.
```
roscd kr_mav_launch/scripts
./demo_gazebo.sh 2
```
 * This will launch 2 robots in gazebo. Do not run more if your machine cannot support.
 * Switch to the `Kill` tab in the `tmux` window and press `Enter` to close everything
 * `dragonfly$ID` namespace is used for each robot. Each vehicle can be interfaced with the quadrotor_control topics/services in this namespace.
 * Use the rqt_multi_mav_manager GUI to control all the robots. Wait until you see `==== Multi MAV Manager is ready for action ===` in one of the tmux pane.
 * You can also use the MATLAB interface to control the robots. Might have to update the `odom_topic` in the interface.