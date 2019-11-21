#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Int64
from std_msgs.msg import Float32MultiArray

def send_data():
    rospy.init_node('playback_node',anonymous = True)
    pub = rospy.Publisher('recorded_points',Float32MultiArray,queue_size=1)
    # ----------------------------------------------------------
    #             Init some variables
    xy_list = []

    # ----------------------------------------------------------
    #            Open file
    f = open("/media/nvidia/JestonSSD-480/catkin_ws/src/sampling/src/samples.txt","r")

    # ----------------------------------------------------------
    #           Loop through the file and append to list
    while True:
        list_as_string = f.readline()
        if list_as_string == "":
            break
        some_var = list_as_string.split()
        xy_list.append([float(some_var[0]),float(some_var[1])])
    f.close()

    # ---------------------------------------------------------
    #          convert to std_msgs.msg.Float32MultiArray
    vec = np.asarray(xy_list)
    vec = np.reshape(vec,(1,np.size(vec)))
    float_vec = Float32MultiArray((1,np.size(vec)),vec)

    msg = Float32MultiArray()
    msg.data = float_vec.data[0]
    # ---------------------------------------------------------
    #          Send the list over ROS
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
if __name__ == '__main__':
    send_data()
