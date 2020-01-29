Usage
-----

Assuming quadrotor_control is checked out into a workspace with arl-unity-ros repository and dependencies.

In separate terminals run:

1. Launch empty simulator
``` bash
$ roslaunch arl_unity_ros simulator.launch param_file:=$(rospack find arl_unity_ros)/config/overpasscity.yaml
```

2. Spawn rosflight quadrotor

``` bash
$ roslaunch arl_unity_ros_air quadrotor.launch x:=-1
```

3. Run example launch for quadrotor_control + rosflight_interface

``` bash
roslaunch rosflight_example example.launch
```

4. Run `mav_manager/scripts/sample.bash`, hit enter to start takeoff (nothing will happen at this point)

``` bash
$ cd $(catkin locate mav_manager)/scripts
$ ./sample.bash
Enable motors...
success: True
message: "Motors on"
Press [Enter] to takeoff
Takeoff...
```

5. Run script to put rosflight in armed state with offboard control enabled

``` bash
$ rosrun arl_unity_ros_air rosflight_offboard.py __ns:=quadrotor
```

