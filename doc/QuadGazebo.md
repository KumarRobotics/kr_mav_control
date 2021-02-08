Various packages exists to simulate a quadrotor in gazebo. [RotorS](https://github.com/ethz-asl/rotors_simulator), [PX4SITL](https://github.com/PX4/sitl_gazebo), [Hector](https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor), etc are some of the few.

We use our own simplified gazebo plugins/simulator (https://github.com/KumarRobotics/mrsl_quadrotor)


```
roslaunch mrsl_quadrotor_launch gazebo.launch world:=levine
roslaunch mrsl_quadrotor_launch spawn.launch mav_type:=pelican mav_name:=juliett
```

This spawns a quadrotor with plugins for `odom` and `so3_cmd` under `juliett` namespace. kr_mav_control plugins have to be launched under this namespace.

```
roslaunch mrsl_quadrotor_launch controller.launch odom_topic:=ground_truth/odom mav_type:=pelican mav_name:=juliett mass:=0.5

```

Launch the GUI to control the robot. Change robot name to `juliett`
```
rosrun rqt_mav_manager rqt_mav_manager
```