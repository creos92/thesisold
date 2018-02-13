#!/bin/bash
source ./devel/setup.bash
chmod +rx ./start_rviz.sh ./start_talker.sh /Server/certificati/server/start_server.sh
cd /Server/certificati/server && ./start_server.sh &
export ROS_IP=10.8.0.1
export ROS_MASTER_URI=http://10.8.0.1:11311
cd /inmoov_ros && ./start_rviz.sh & ./start_talker.sh

