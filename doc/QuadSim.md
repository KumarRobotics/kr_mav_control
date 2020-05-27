## Example with GUI
```
roslaunch kr_mav_launch rviz.launch
roslaunch kr_mav_launch demo.launch sim:=true vicon:=false mav_name:=quadrotor
```

Run a simple example script
```
roscd kr_mav_launch/scripts
./sample.bash quadrotor
```

There is also a GUI that can be used to send simple commands to the robot through the `rqt_mav_manager`. Launch it by running
```
rosrun rqt_mav_manager rqt_mav_manager
```
then try Motors ON -> Take Off -> Go To (set z > 0)

## Example with Waypoint Navigation Tool

Clone the [waypoint_navigation_tool](https://github.com/KumarRobotics/waypoint_navigation_plugin) in your workspace.

```
roslaunch kr_mav_launch rviz.launch
roslaunch kr_mav_launch demo.launch sim:=true vicon:=false mav_name:=quadrotor
rosrun kr_trackers waypoints_to_action.py __ns:=quadrotor
```

Use rqt to start motors and takeoff.
```
rosrun rqt_mav_manager rqt_mav_manager
```
 * then try Motors ON -> Take Off -> Go To (set z > 0)

Use rviz to place waypoints and publish on the topic `/waypoints`. The `waypoints_to_action` node listens to this topic, sends an action goal to `TrajectoryTracker` and switches the tracker.
