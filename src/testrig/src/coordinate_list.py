#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Int64, Int8

##-------------------------DESCRIPTION------------------------------##
## Converts the path object to coordinate points the action decider can handle
##-------------------------DESCRIPTION------------------------------##

##-----------------------------INIT---------------------------------##
rospy.init_node('coordinate_list', anonymous=True)
pubNextPoint = rospy.Publisher('goal_map', PointStamped, queue_size = 1)
rate = rospy.Rate(10)
pointList = []
##-----------------------------INIT---------------------------------##


def convertList(theList):
	pointList = []
	for row in theList:
		tmpPoint = PointStamped()	
		tmpPoint.point.x = row[0]
		tmpPoint.point.y = row[1]
		tmpPoint.point.z = 0
		pointList.append(tmpPoint)
	return pointList

def nextGoal(msg):
	global pointList


	if (len(pointList) > 0) and (msg.data == 1):
		nextPoint = pointList.pop(0)
		#print("Next target: ")
		#print(nextPoint.x)
		#print(nextPoint.y)
		nextPoint.header.frame_id = 'map'
		pubNextPoint.publish(nextPoint)


def addGoal(msg):
	global pointList
	
	for iterate_point in msg.poses:
	    tmpPoint = PointStamped()
	    tmpPoint.point.x = iterate_point.pose.position.x
	    tmpPoint.point.y = iterate_point.pose.position.y
	    tmpPoint.point.z = 0
	    pointList.append(tmpPoint)
	
	
	

def main():
	global pointList

	origo = PointStamped()
	origo.point.x = 0
	origo.point.y = 0
	pubNextPoint.publish(origo)

	subMotorAction = rospy.Subscriber('goal_reached', Int64, nextGoal)
	subRvizGoal = rospy.Subscriber('Astar_path', Path, addGoal)

	while not rospy.is_shutdown():		
		rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
