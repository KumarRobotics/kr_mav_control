#!/bin/bash

ROBOT="quadrotor"

echo "Enable motors..."
rosservice call /$ROBOT/motors true
sleep 1

read -p "Press [Enter] to takeoff"
echo "Takeoff..."
rosservice call /$ROBOT/takeoff
sleep 1

read -p "Press [Enter] to go to [1, 1, 1]"
rosservice call /$ROBOT/goTo "goal: [1.0, 1.0, 1.0, 0.0]"
sleep 1

read -p "Press [Enter] to hover"
rosservice call /$ROBOT/hover
sleep 1

read -p "Press [Enter] to go home"
rosservice call /$ROBOT/goHome
sleep 1

read -p "Press [Enter] to land"
rosservice call /$ROBOT/land
sleep 1

read -p "Press [Enter] to estop"
rosservice call /$ROBOT/estop
