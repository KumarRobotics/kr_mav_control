#!/bin/bash

ROBOT="WoodPixhawkUAV"

echo "Enable motors..."
rosservice call /motors true
sleep 1

read -p "Press [Enter] to takeoff"
echo "Takeoff..."
rosservice call /takeoff
sleep 1

read -p "Press [Enter] to go to [1, 1, 1]"
rosservice call /goTo "goal: [1.0, 1.0, 1.0, 0.0]"
sleep 1

read -p "Press [Enter] to hover"
rosservice call /hover
sleep 1

read -p "Press [Enter] to go home"
rosservice call /goHome
sleep 1

read -p "Press [Enter] to estop"
rosservice call /estop
