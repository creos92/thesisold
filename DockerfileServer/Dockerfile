FROM osrf/ros:indigo-desktop-full

RUN     \
        svn checkout https://github.com/creos92/thesis.git/trunk/Server && \
        svn checkout https://github.com/creos92/thesis.git/trunk/inmoov_ros && \
        apt-get update && apt-get install build-essential checkinstall libssl-dev liblzo2-dev libpam0g-dev  wget -y && \
        wget https://swupdate.openvpn.org/community/releases/openvpn-2.4.4.tar.gz && sudo tar zxvf openvpn-2.4.4.tar.gz && cd openvpn-2.4.4 && ./configure && make && checkinstall -y && \
        /bin/bash -c "source /opt/ros/indigo/setup.bash && cd /inmoov_ros && catkin_make"

CMD cd inmoov_ros && chmod +rx start_s.sh && sync &&./start_s.sh
