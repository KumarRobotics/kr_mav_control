Launch
```
roslaunch quadrotor_simulator rviz.launch
roslaunch mav_manager demo.launch sim:=true vicon:=false
```

Run a simple example script
```
cd quadrotor_control/mav_manager/scripts
./sample.bash
```

There is also a GUI that can be used to send simple commands to the robot through the `mav_manager`. Launch it by running
```
rosrun rqt_mav_manager rqt_mav_manager
```
then try Motors ON -> Take Off -> Go To (set z > 0)

TODO - Waypoint Navigation
