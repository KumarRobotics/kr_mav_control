## Example with multiple simulated robots

Clone and build [kr_multi_mav_manager](https://github.com/KumarRobotics/multi_mav_manager) in your workspace

Helper bash scripts are added to launch multiple robots. (Requires tmux installed)
```
roscd kr_multi_mav_manager/scripts
./demo_gazebo.sh 2
```
 * This will launch 2 robots in gazebo. Do not run more if your machine cannot support.
 * Switch to the `Kill` tab in the `tmux` window and press `Enter` to close everything
 * `dragonfly$ID` namespace is used for each robot. Each vehicle can be interfaced with the kr_mav_control topics/services in this namespace.
 * Use the rqt_multi_mav_gui GUI to control all the robots. Wait until you see `==== Multi MAV Manager is ready for action ===` in one of the tmux pane.
 * You can also use the MATLAB interface to control the robots. Might have to update the `odom_topic` in the interface.

If MAVs need to be loaded in pre-defined start locations, create a CSV file to input X,Y,Z,YAW on each new line per MAV. An example csv is provided.
```
roscd kr_multi_mav_manager/scripts
./demo_gazebo.sh start_locations.csv
```
