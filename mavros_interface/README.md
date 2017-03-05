# mavros_interface

## `mavros_interface_nodelet`

This nodelet converts `quadrotor_msgs/SO3Command` to `mavros_msgs/AttitudeTarget`.

## `attitude_tuner_tool`

This node computes the geodesic error between `mavros_msgs/AttitudeTarget` with `geometry_msgs/Pose` to give the user information about the attitude response. To view the geodesic error between b3 and b3_desired, simply launch

```
roslaunch mavros_interface attitude_tuning_tool.launch setpoint_raw_attitude:=/quadrotor/mavros/setpoint_raw/attitude pose:=/quadrotor/mavros/local_position/pose
```

and run

```
rqt_plot /attitude_tuning_tool/b3_geodesic_error
```

#### `mavros_msgs` requirement

These nodes require `mavros_msgs` to be present when building. If `mavros_msgs` is not found when building the first time, a warning is given and nothing in this package is built. If `mavros_msgs` is installed after this, force a recheck by adding `--force-cmake` to the `catkin_make`/`catkin build` command.
