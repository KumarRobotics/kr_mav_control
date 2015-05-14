#!/bin/bash

echo "Enable motors..."
rosservice call /quadrotor/motors true
sleep 1

echo "Takeoff..."
rosservice call /quadrotor/takeoff
sleep 4

echo "goTo [0.0, 0.0, 1.0, 0.0]"
rosservice call /quadrotor/goTo "goal: [0.0, 0.0, 1.0, 0.0]"
sleep 5

echo "setDesVelWorld 'goal: [0.0, 0.0, 1.0, 0.0]'"
rosservice call /quadrotor/setDesVelWorld "goal: [0.0, 0.0, 1.0, 0.0]"
sleep 2

echo "setDesVelWorld 'goal: [0.0, 0.0, 3.0, 0.0]'"
rosservice call /quadrotor/setDesVelWorld "goal: [0.0, 0.0, 3.0, 0.0]"
sleep 2

echo "Hover..."
rosservice call /quadrotor/hover
sleep 3

echo "setDesVelBody 'goal: [0.0, 0.0, -3.0, 1.0]'"
rosservice call /quadrotor/setDesVelBody "goal: [0.0, 0.0, -3.0, 1.0]"
sleep 2

echo "eHover..."
rosservice call /quadrotor/ehover
