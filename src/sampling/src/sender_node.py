#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64
from std_msgs.msg import String
import time


def send_data():
    rospy.init_node('sender_node',anonymous = True)
    pub1 = rospy.Publisher('a_topic',Int64,queue_size=1)
    rate = rospy.Rate(1) #0.2Hz
    I = Int64()
    I = 5
    while not rospy.is_shutdown():
        pub1.publish(I)
        I = I + 1
        rate.sleep()


if __name__ == '__main__':
    send_data()
