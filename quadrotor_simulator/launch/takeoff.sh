#!/bin/bash

rostopic pub -1 /quadrotor/motors std_msgs/Bool 0
rostopic pub -1 /quadrotor/controllers_manager/line_tracker/goal line_tracker/Goal "{x: 0.0, y: 0.0, z: 2.0}"
rosservice call /quadrotor/controllers_manager/transition line_tracker/LineTracker
rostopic pub -1 /quadrotor/motors std_msgs/Bool 1
