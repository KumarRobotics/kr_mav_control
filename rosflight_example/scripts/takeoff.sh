#!/bin/bash

ROBOT=$1

rosservice call /$ROBOT/mav_services/takeoff
rosrun arl_unity_ros_air rosflight_offboard.py __ns:="$ROBOT"
