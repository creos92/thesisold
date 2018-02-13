#!/bin/bash
echo Open Rviz
sleep 5s
source ./devel/setup.bash
roslaunch inmoov_description display.launch model:=./src/inmoov_description/robots/inmoov_right_hand.urdf.xacro 
