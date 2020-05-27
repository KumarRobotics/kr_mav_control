#!/bin/bash

if [ $# -eq 0 ]; then
  echo "Input mav name as argument"
  exit 1
fi
MAV_NAME=$1

rostopic pub -1 /$MAV_NAME/motors std_msgs/Bool 0
rostopic pub -1 /$MAV_NAME/trackers_manager/line_tracker_min_jerk/goal kr_mav_msgs/LineTrackerGoal "{x: 0.5, y: 0.5, z: 2.0, yaw: 0.0, v_des: 0.0, a_des: 0.0}"
rosservice call /$MAV_NAME/trackers_manager/transition kr_trackers/LineTrackerMinJerk
rostopic pub -1 /$MAV_NAME/motors std_msgs/Bool 1
