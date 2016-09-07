#!/bin/bash

source ~/.bashrc

ROBOT=quadrotor

echo -e "\nEnabling motors ..."
rosservice call /$ROBOT/mav_manager_node/motors true

echo -e "\nArming ..."
rosservice call /$ROBOT/mavros/cmd/arming 1

echo -e "\nSetting mode to OFFBOARD ..."
rosservice call /$ROBOT/mavros/set_mode "{base_mode: 0, custom_mode: 'offboard'}"

echo -e "\nTaking off ..."
rosservice call /$ROBOT/mav_manager_node/takeoff
