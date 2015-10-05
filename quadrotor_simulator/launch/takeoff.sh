#!/bin/bash

rostopic pub -1 /quadrotor/motors std_msgs/Bool 0
rostopic pub -1 /quadrotor/trackers_manager/line_tracker_min_jerk/goal quadrotor_msgs/LineTrackerGoal "{x: 0.5, y: 0.5, z: 2.0, yaw: 0.0, v_des: 0.0, a_des: 0.0}"
rosservice call /quadrotor/trackers_manager/transition std_trackers/LineTrackerMinJerk
rostopic pub -1 /quadrotor/motors std_msgs/Bool 1
