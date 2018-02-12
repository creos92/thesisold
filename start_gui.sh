#!/bin/bash
echo Interfaccia Joint
sleep 10s
rosparam set use_gui true
rosrun joint_state_publisher joint_state_publisher
