# kr_mavros_interface

This node translates `kr_mav_msgs/SO3Command` to `mavros_msgs/AttitudeTarget`.

Please note that your flight controller yaw orientation estimation may be
different to your odom system orientation estimation. We should transform our
`kr_mav_msgs/SO3Command` into the flight controller yaw before publishing it.

#### `mavros_msgs` requirement

This node requires `mavros_msgs` to be present when building. If `mavros_msgs` is not found when building the first time, a warning is given and nothing in this package is built. If `mavros_msgs` is installed after this, force a recheck by adding `--force-cmake` to the `catkin_make`/`catkin build` command.
