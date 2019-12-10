#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64 #used to import desired turn radius value
from geometry_msgs.msg import Point #x, y used for front and rear waist angle (float 64) respectively

##-----------------------------INIT---------------------------------##
rospy.init_node('turn_node')
pubAngles = rospy.Publisher('cmd_waist_twists', Point, queue_size = 1)
rate = rospy.Rate(10)


##------------------------------------------------------------------##

xbox = 0
frontAngle = 0
rearAngle = 0

def xboxTakeover(msg):
	global xbox
	xbox = msg.data

def motorActionCallback(msg):
	motor_action = msg.data
	global frontAngle
	global rearAngle
	global turnRate
	if (motor_action == 0 or motor_action == 6): # turn left
		if abs(frontAngle) < 45: 
			frontAngle = -45
			rearAngle = frontAngle
	elif (motor_action == 2 or motor_action == 8):	#turn right
		if abs(frontAngle) < 45: 
			frontAngle = 45
			rearAngle = frontAngle
	elif (motor_action == 1 or motor_action == 7):	#straight
		frontAngle = 0
		rearAngle = 0

def doStuff():
	global xbox
	global frontAngle
	global rearAngle

	angles_msg = Point()
	angles_msg.x = frontAngle
	angles_msg.y = rearAngle
	angles_msg.z = 0
	if (xbox == 0):
		pubAngles.publish(angles_msg)
	


def main():
	sub_motorAction = rospy.Subscriber('motor_action', Int64, motorActionCallback)
	sub_xboxTakeover = rospy.Subscriber('xbox_takeover', Int64, xboxTakeover)
	while not rospy.is_shutdown():
		doStuff()
		rate.sleep()

if __name__ == '__main__':

	try:
		main()
	except rospy.ROSInterruptException:
		pass
