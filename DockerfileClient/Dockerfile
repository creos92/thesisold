FROM osrf/ros:indigo-desktop-full

RUN /bin/bash -c "svn checkout https://github.com/creos92/thesis.git/trunk/Client"

RUN /bin/bash -c "svn checkout https://github.com/creos92/thesis.git/trunk/inmoov_ros"

RUN /bin/bash -c "apt-get update && apt-get install build-essential checkinstall libssl-dev liblzo2-dev libpam0g-dev  wget -y"

RUN /bin/bash -c "wget https://swupdate.openvpn.org/community/releases/openvpn-2.4.4.tar.gz && sudo tar zxvf openvpn-2.4.4.tar.gz && cd openvpn-2.4.4 && ./configure && make && checkinstall -y"

RUN /bin/bash -c "source /opt/ros/indigo/setup.bash && cd /inmoov_ros && catkin_make"

#CMD cd /certificati/client1 && openvpn --config client.conf 
#CMD cd inmoov_ros && chmod +rx start.sh && ./start.sh


