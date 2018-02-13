#!/bin/bash
source ./devel/setup.bash
chmod +rx start_play.sh /Client/certificati/client1/start_client.sh
cd /Client/certificati/client1 && ./start_client.sh &
sleep 15s
export ROS_IP=10.8.0.6
export ROS_MASTER_URI=http://10.8.0.1:11311
cd /inmoov_ros && ./start_play.sh
