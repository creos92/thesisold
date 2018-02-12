#!/bin/bash
chmod /inmoov_ros/start_rviz.sh 
cd  inmoov_ros  && 
./start_rviz.sh
./start_talker.sh
# &  gnome-terminal -x ./start_gui.sh
