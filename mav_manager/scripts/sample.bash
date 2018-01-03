#!/bin/bash

ROBOT="quadrotor"

echo "Enable motors..."
rosservice call /$ROBOT/mav_services/motors true
sleep 1

read -p "Press [Enter] to takeoff"
echo "Takeoff..."
rosservice call /$ROBOT/mav_services/takeoff
sleep 1

read -p "Press [Enter] to go to [1, 1, 1]"
rosservice call /$ROBOT/mav_services/goTo "goal: [1.0, 1.0, 1.0, 0.0]"
sleep 1

read -p "Press [Enter] to hover"
rosservice call /$ROBOT/mav_services/hover
sleep 1

read -p "Press [Enter] to ascend at 0.5 m/s"
rosservice call /$ROBOT/mav_services/setDesVelInWorldFrame "goal: [0.0, 0.0, 0.5, 0.0]"
sleep 1

read -p "Press [Enter] to hover"
rosservice call /$ROBOT/mav_services/hover
sleep 1

read -p "Press [Enter] to go home"
rosservice call /$ROBOT/mav_services/goHome
sleep 1

read -p "Press [Enter] to land"
rosservice call /$ROBOT/mav_services/land
sleep 1

read -p "Press [Enter] to estop"
rosservice call /$ROBOT/mav_services/estop
