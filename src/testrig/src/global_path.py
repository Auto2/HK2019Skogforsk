#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8

"""
Name: global_path.py (node)
Date: 2019-11-28
Description: This node is used for two purpuses
			 1) To store global goals that are generated on the topic
   				/move_base_simple/goal in a list called goalList.
			 2) To provide the planning node (astar.py) with goals from 
			   	goalList one at a time. 
			Purpuse 2 shall only be fullfilled once coordinate_list.py 
			has made a new request for a new global goal.

"""

##-----------------------------INIT---------------------------------##
rospy.init_node('global_path_node', anonymous=True)
pubGlobalPathNext = rospy.Publisher('next_goal_global_path', PoseStamped, queue_size = 1)
rate = rospy.Rate(10)
requestHasBeenMade = 0 #1 if request (for new global goal) has been made , 0 if request has not been made
goalList = []
initialPoint = 1
##-----------------------------INIT---------------------------------##

def addGoal(msg):
	global goalList
	global initialPoint
	goalList.append(msg)
	print('# appended: ' + str(len(goalList)))
	tmpGoal = goalList.pop(0)
	pubGlobalPathNext.publish(tmpGoal)
	#if (len(goalList) == 1) and (initialPoint == 1):#publish immediately if it's the initial point	
	#	initialPoint = 0
	#	tmpGoal = goalList.pop(0)
	#	pubGlobalPathNext.publish(tmpGoal)

def sendGoal(msg):
	global goalList
	global requestHasBeenMade
	#requestHasBeenMade = msg.data #1 for wanting the thing, 0 not wanting the thing

	if ((msg.data == 1) and (requestHasBeenMade == 0)):	#If coordinate_list (node) wants a new goal to be sent to astar for planning (node)
		print("GLOBAL PATH: path request has been made")
		requestHasBeenMade = 1
		if len(goalList) > 0:
			tmpGoal = goalList.pop(0)
			pubGlobalPathNext.publish(tmpGoal)

	elif ((msg.data == 0) and (requestHasBeenMade == 1)):	#Allows global_path (node) to again accept coordinate_list requests for new goals
		requestHasBeenMade = 0
		print("GLOBAL PATH: not requesting path ")
	#print('# appended: ' + str(len(goalList)))

def main():

	subRvizGoal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, addGoal)
	subGoalRequest = rospy.Subscriber('/request_next_global_goal', Int8, sendGoal)
	while not rospy.is_shutdown():
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
