# The Simple and Compound Lissajous Trackers

This document exists to give you a sense of how to use the simple
and compound lissajous trackers. These trackers are used to create
3D Lissajous trajectories and sums of such trajectories.

## Mathematical Trajectory Description and Parameters

The general form of the 3D Lissajous trace, parameterized
by the variable `s`, is given as follows:

```
x(s) = x_amp * (1 - cos(2 * PI * x_num_periods * s / period))
y(s) = y_amp * sin(2 * PI * y_num_periods * s / period)
z(s) = z_amp * sin(2 * PI * z_num_periods * s / period)
yaw(s) = yaw_amp * sin(2 * PI * yaw_num_periods * s / period)
```

The trajectory is shaped by shaping the `x_amp`, `y_amp`, `z_amp`, `yaw_amp`,
`x_num_periods`, `y_num_periods`, `z_num_periods`, `yaw_num_periods`, and
`period` parameters.

Since such a trajectory does not ramp up and ramp down smoothly, the
trajectory is reparameterized by a smooth ramp-up and ramp-down function
given by an 8th order polynomial. The ramp-up function is paramterized by
the ramp time `ramp_time`.

Finally, te trajectory may be repeated. This is the function of the
`num_cycles` parameter.

## Commanding a Lissajous

The Lissajous tracker and adder may be called by the approprate MAV services
(lissajous and compound, respectively). For example, a normal Lissajous:

```
rosservice call /quadrotor_name/mav_services/lissajous "{x_amp: 1.25, y_amp: 1.25, z_amp: 0.75, yaw_amp: 3.1415, x_num_periods: 12, y_num_periods: 12, z_num_periods: 16, yaw_num_periods: 5, period: 60, num_cycles: 1, ramp_time: 2}"
```
