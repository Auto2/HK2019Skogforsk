#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import time
import math

i = 1


def save_sample(data):
    global i
    x = data.pose.position.x
    y = data.pose.position.y
    
    i = i + 1
    if i == 50:
        f = open("/media/nvidia/JestonSSD-480/catkin_ws/src/sampling/src/samples.txt","a")
        f.write("%f %f\n" % (x, y))
        f.close()
        i = 1

def sampling():
    rospy.init_node('sampling_node',anonymous = True)
    sub1 = rospy.Subscriber('/zed/zed_node/pose',PoseStamped,save_sample)
    rospy.spin()

if __name__ == '__main__':
    f = open("/media/nvidia/JestonSSD-480/catkin_ws/src/sampling/src/samples.txt","w")
    f.write("")
    f.close()
    sampling()
