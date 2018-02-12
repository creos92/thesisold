#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy,sys
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def talker():
    pub = rospy.Publisher('joint_command', JointState, queue_size=10)
    rospy.init_node('comando', anonymous=True)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        if sys.argv[1] == "241":
        #da modificare
            right_thumb1= +1.125 - float(sys.argv[7])/180*1.125

            right_thumb= 1.5 - float(sys.argv[2])/180*1.5
            right_thumb3= 1.5 - float(sys.argv[2])/180*1.5

            right_index1= 1.5 - float(sys.argv[3])/180*1.5
            right_index= 1.5 - float(sys.argv[3])/180*1.5
            right_index3= 1.5 - float(sys.argv[3])/180*1.5

            right_middle1= 1.5 - float(sys.argv[4])/180*1.5
            right_middle= 1.5 - float(sys.argv[4])/180*1.5
            right_middle3= 1.5 - float(sys.argv[4])/180*1.5

        #da modificare
            right_ring1= -0.15 + float(sys.argv[8])/180*0.15

            right_ring=1.5 - float(sys.argv[5])/180*1.5
            right_ring3=1.5 - float(sys.argv[5])/180*1.5
            right_ring4=1.5 - float(sys.argv[5])/180*1.5

        #da modificare
            right_pinky1= -0.15 + float(sys.argv[9])/180*0.15

            right_pinky=1.5 - float(sys.argv[6])/180*1.5
            right_pinky3=1.5 - float(sys.argv[6])/180*1.5
            right_pinky4=1.5 - float(sys.argv[6])/180*1.5

            base_to_hand= -3.15 +  float(sys.argv[10])/180*3.15

            hello_str=JointState()
            hello_str.header=Header()
            hello_str.header.stamp=rospy.Time.now()

            right_thumb1= 0 #+ float(sys.argv[7])/180*1.125
            right_thumb= 1.5  #- float(sys.argv[2])/180*1.5
            right_thumb3= 0 #- float(sys.argv[2])/180*1.5

            hello_str.name=['right_thumb1', 'right_thumb', 'right_thumb3', 'right_index1', 'right_index', 'right_index3', 'right_middle1', 'right_middle', 'right_middle3', 'right_ring1', 'right_ring', 'right_ring3', 'right_ring4', 'right_pinky1', 'right_pinky', 'right_pinky3', 'right_pinky4', 'base_to_hand']
            hello_str.position=[right_thumb1, right_thumb, right_thumb3, right_index1, right_index, right_index3, right_middle1, right_middle, right_middle3, right_ring1, right_ring, right_ring3, right_ring4, right_pinky1, right_pinky, right_pinky3, right_pinky4, base_to_hand]
            hello_str.velocity = []
            hello_str.effort = []
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
