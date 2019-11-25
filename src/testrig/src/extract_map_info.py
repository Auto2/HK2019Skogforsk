#!/usr/bin/env python

import numpy as np
import rospy

from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Path

##-----------------------------DESCRIPTION---------------------------------##
#
##-----------------------------DESCRIPTION---------------------------------##


##-----------------------------INIT---------------------------------##
rospy.init_node('extract_map_info', anonymous=True)
pub_goal = rospy.Publisher('occupied_coordinates_in_map', Point, queue_size = 1) #goal coordinates relative to rig frame
rosRate = 0.2
marker_array = MarkerArray()
obstacle_list = [] #list of occupied points
rate = rospy.Rate(rosRate)
##-----------------------------INIT---------------------------------##

Q[]
black_list[]
goal = Point()
goal.x = 2
goal.y = 2
max_obstacle_height = 0.1
min_obstacle_distance = 0.2

class Node:
	def __init__(self,x=0,y=0,z=0,theta=0,parent=None,heuristic=0,cost_to_come=0):
		self.x = x
		self.y = y
		self.z = z
		self.theta = theta
		self.parent = parent
		self.heuristic = heuristic
		self.cost_to_come = cost_to_come
		self.cost = self.heuristic + self.cost_to_come
		self.representation = [self.x, self.y, self.theta]

	def update_cost(self):
		self.cost = self.heuristic + self.cost_to_come
	

def update_marker(msg):
	global marker_array
	marker_array = msg

def show_occupied_points():
	global marker_array
	#print(marker_array.markers[0])
	for point in marker_array.markers:
		#print(point.points)
		#append points to list
		
def generate_nodes(parent):
	global black_list
	global Q
	black_list.append(parent.representation) #To remember which nodes have been evaluated
	
	L = [-15,0,15]#vinklar 
	step = 0.1
	for angle in L:
		#Vi vill beräkna ett x och ett y för att öppna ny nod i
		x_temp = np.cos(parent.theta + angle)
		y_temp = np.sin(parent.theta + angle)
		xn = x_temp + parent.x
		yn = y_temp + parent.y
		thetan = parent.theta+angle
		#if child is suitable to be created (not already visited, not in obstacle etc)
		if is_node_ok(xn, yn, thetan):
			child = Node(xn,yn,0,thetan)
			Q.append(child)
		else:
			pass

def is_node_ok(x, y, theta): #checks if node is suitable to be created
    global obstacle_list
	global black_list
	global max_obstacle_height
	global min_obstacle_distance
	#finns noden i obstacleList?

	for obstacle in obstacle_list:
		#kanske ta hänsyn till orienteringen?
		if obstacle.z < max_obstacle_height: #Max obstacle height
			distance = np.sqrt(np.square(x - obstacle.x)+np.square(y - obstacle.y))
			if distance > min_obstacle_distance:			
				return True
		
		else:
			return False
	return True

def notTooCloseToObstacle(obstacle_list, x, y):
	no_collision = True #naive default expectation

def getBestNode():
	global Q
	bestIndex = 0
	currentBest = Q[bestIndex]
	for i in range(len(Q)):
		Q[i].update_cost()
		if Q[i].cost < currentBest:
			currentBest = Q[i]
			bestIndex = i
	bestNode = Q.pop(bestIndex)
	return bestNode

def closeEnoughTogoal(node): #returns true if close enough to the goal
	global goal
	distance = np.sqrt(np.square(node.x - goal.x)+np.square(node.y - goal.y))
	if distance < 0.05:
		return True
	else:
		return False
	

def A_star:
	global Q
	while Q:
		#get bestnode
		bestNode = getBestNode()

		#check if goal
		if closeEnoughTogoal(bestNode):
			#return success and the path
			pass
		else:
			#keep searching
			pass
			
			generate_nodes(parent)

def main():
	global black_list
	global Q
	sub_markerArray_map = rospy.Subscriber('occupied_cells_vis_array', MarkerArray, update_marker) #subscribes to the marker array which contains detected physical points in space
	origin = Node()
	Q.append(origin)
	
	while not rospy.is_shutdown():
		#main function here	
		rate.sleep()
		A_star()
		#show_occupied_points()

if __name__ == '__main__':

    try:
        main()
    except rospy.ROSInterruptException:
        pass
