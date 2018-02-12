#!/usr/bin/env python
import rospy,sys
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Header
from serial_bridge.msg import generic_serial

pub = rospy.Publisher('joint_command', JointState, queue_size=10)


def fnc_callback(msg):
    hand_pos=JointState()
    data = [ord(x) for x in msg.msg]
    if data[0] == 241:
    #da modificare
        right_thumb1= +1.125 - float(data[6])/180*1.125

        right_thumb= 1.5 - float(data[1])/180*1.5
        right_thumb3= 1.5 - float(data[1])/180*1.5

        right_index1= 1.5 - float(data[2])/180*1.5
        right_index= 1.5 - float(data[2])/180*1.5
        right_index3= 1.5 - float(data[2])/180*1.5

        right_middle1= 1.5 - float(data[3])/180*1.5
        right_middle= 1.5 - float(data[3])/180*1.5
        right_middle3= 1.5 - float(data[3])/180*1.5

    #da modificare
        right_ring1= -0.15 + float(data[7])/180*0.15

        right_ring=1.5 - float(data[4])/180*1.5
        right_ring3=1.5 - float(data[4])/180*1.5
        right_ring4=1.5 - float(data[4])/180*1.5

    #da modificare
        right_pinky1= -0.15 + float(data[8])/180*0.15

        right_pinky=1.5 - float(data[5])/180*1.5
        right_pinky3=1.5 - float(data[5])/180*1.5
        right_pinky4=1.5 - float(data[5])/180*1.5

        base_to_hand= -3.15 +  float(data[9])/180*3.15

        hand_pos.name=['right_thumb1', 'right_thumb', 'right_thumb3', 'right_index1', 'right_index', 'right_index3', 'right_middle1', 'right_middle', 'right_middle3', 'right_ring1', 'right_ring', 'right_ring3', 'right_ring4', 'right_pinky1', 'right_pinky', 'right_pinky3', 'right_pinky4', 'base_to_hand']
        hand_pos.position=[right_thumb1, right_thumb, right_thumb3, right_index1, right_index, right_index3, right_middle1, right_middle, right_middle3, right_ring1, right_ring, right_ring3, right_ring4, right_pinky1, right_pinky, right_pinky3, right_pinky4, base_to_hand]
        hand_pos.velocity = []
        hand_pos.effort = []
        hand_pos.header=Header()
        hand_pos.header.stamp=rospy.Time.now()
        rospy.loginfo(hand_pos)
        pub.publish(hand_pos)

def talker():
    global pub
    sub = rospy.Subscriber('/output/serial_topic', generic_serial, fnc_callback)
    rospy.init_node('comando', anonymous=True)
    rospy.spin()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
