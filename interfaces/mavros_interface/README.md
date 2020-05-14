# mavros_interface

This node translates `kr_quadrotor_msgs/SO3Command` to `mavros_msgs/AttitudeTarget`.

#### `mavros_msgs` requirement

This node requires `mavros_msgs` to be present when building. If `mavros_msgs` is not found when building the first time, a warning is given and nothing in this package is built. If `mavros_msgs` is installed after this, force a recheck by adding `--force-cmake` to the `catkin_make`/`catkin build` command.
