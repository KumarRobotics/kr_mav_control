#!/bin/bash

if [ $# -eq 0 ]; then
  MAV_NAME=quadrotor
  echo "Using default mav_name $MAV_NAME"
else
  MAV_NAME=$1
fi

echo "Enable motors..."
rosservice call /$MAV_NAME/mav_services/motors true
sleep 1

read -p "Press [Enter] to takeoff"
echo "Takeoff..."
rosservice call /$MAV_NAME/mav_services/takeoff
sleep 1

read -p "Press [Enter] to go to [1, 1, 1]"
rosservice call /$MAV_NAME/mav_services/goTo "goal: [1.0, 1.0, 1.0, 0.0]"
sleep 1

read -p "Press [Enter] to hover"
rosservice call /$MAV_NAME/mav_services/hover
sleep 1

read -p "Press [Enter] to ascend at 0.5 m/s"
rosservice call /$MAV_NAME/mav_services/setDesVelInWorldFrame "goal: [0.0, 0.0, 0.5, 0.0]"
sleep 1

read -p "Press [Enter] to hover"
rosservice call /$MAV_NAME/mav_services/hover
sleep 1

read -p "Press [Enter] to go home"
rosservice call /$MAV_NAME/mav_services/goHome
sleep 1

read -p "Press [Enter] to land"
rosservice call /$MAV_NAME/mav_services/land
sleep 1

read -p "Press [Enter] to estop"
rosservice call /$MAV_NAME/mav_services/estop
