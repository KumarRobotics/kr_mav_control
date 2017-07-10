
Launch 
```
roslaunch mav_manager demo.launch sim:=true vicon:=false
roslaunch quadrotor_simulator rviz.launch
```

Run a simple example script
```
cd quadrotor_control/mav_manager/scripts
./sample.bash
```
More advanced GUI is available at https://github.com/KumarRobotics/kr_ui. Clone and then compile the workspace again. 

`rosrun rqt_quadrotor_safety rqt_quadrotor_safety`

then try MotorsON -> TakeOff -> GoTo (set z other than 0)

TODO - Waypoint Navigation
