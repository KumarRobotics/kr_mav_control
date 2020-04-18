#!/bin/bash

if [ $# -eq 0 ]; then
  echo "Input mav name as argument"
  exit 1
fi
ROBOT=$1

echo "Enable motors... ${ROBOT}"
rosservice call /$ROBOT/mav_services/motors true
sleep 1

echo "Takeoff... ${ROBOT}"
rosservice call /$ROBOT/mav_services/takeoff
sleep 1