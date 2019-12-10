#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

rospy.init_node('odom_pub')


odom_pub = rospy.Publisher('/my_odom', Odometry,queue_size=1)

rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

odom = Odometry() #instance of messge type odometry
header = Header()
header.frame_id = '/odom'

model = GetModelStateRequest()
model.model_name = 'testrig'

rate = rospy.Rate(10)

while not rospy.is_shutdown():
	
	result = get_model_srv(model) #odom info in gazebo

	odom.pose.pose = result.pose
	odom.twist.twist = result.twist

	header.stamp = rospy.Time.now()
	odom.header = header

	odom_pub.publish(odom)

	rate.sleep()



# see https://www.youtube.com/watch?v=I_5leJK8vhQ

