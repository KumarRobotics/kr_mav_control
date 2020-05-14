#!/bin/bash

if [ $# -eq 0 ]; then
  echo "Input mav name as argument"
  exit 1
fi
ROBOT=$1

rostopic pub -1 /$ROBOT/motors std_msgs/Bool 0
rostopic pub -1 /$ROBOT/trackers_manager/line_tracker_min_jerk/goal kr_quadrotor_msgs/LineTrackerGoal "{x: 0.5, y: 0.5, z: 2.0, yaw: 0.0, v_des: 0.0, a_des: 0.0}"
rosservice call /$ROBOT/trackers_manager/transition std_trackers/LineTrackerMinJerkAction
rostopic pub -1 /$ROBOT/motors std_msgs/Bool 1
