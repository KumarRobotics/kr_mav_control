#!/bin/bash

ROBOT="quadrotor"
N=5

for i in `seq 1 $N`;
do
  echo "$ROBOT$i> Enable motors..."
  rosservice call /$ROBOT$i/mav_services/motors true
done;

read -p "Press [Enter] to takeoff"

for i in `seq 1 $N`;
do
  echo "$ROBOT$i> Takeoff..."
  rosrun rosflight_example takeoff.sh "$ROBOT$i" &
done;

echo "Done!"         
