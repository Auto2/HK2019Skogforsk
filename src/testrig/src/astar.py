#!/usr/bin/env python

import numpy as np
import rospy

from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from std_msgs.msg import Int8, Int64
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion

##-----------------------------DESCRIPTION---------------------------------##
# Stores the global path. Computes an A* path between the global coordinates.
##-----------------------------DESCRIPTION---------------------------------##


##-----------------------------INIT---------------------------------##
rospy.init_node('astar', anonymous=True)
pub_goal = rospy.Publisher('occupied_coordinates_in_map', Point, queue_size = 1) #goal coordinates relative to rig frame
pub_goal = rospy.Publisher('goal_rig', Point, queue_size = 1) 

marker_array = MarkerArray()
obstacle_list = [] #list of occupied points

pub_path = rospy.Publisher('Astar_path',Path,queue_size=1)
path_is_ok = rospy.Publisher('path_is_ok',Int8,queue_size=1)

Q=[]
black_list=[]
the_path = Path()
globalGoalList = []

the_path.header.frame_id = 'map'
goal = Twist()
pathGoal = Twist()

#goal.x = -8
#goal.y = -5

currentPosition = Twist()
#currentTheta = 0
max_obstacle_height = 2
min_obstacle_height = 0.1
min_obstacle_distance = 0.4
#i = 0
initialPoint = 1

planCloseEnough = 0.35	# when to start planning for next point, must be larger than closeEnough in action_decider_node.py

planStep = 0.4
planAngle = [-0.2, -0.1, 0, 0.1, 0.2]
A_star_iterations = 100

poseCloseEnoughToGoal = 0.4	# when planned goal is close enough to desired goal
angleCloseEnoughToGoal = 0.7	# when planned goal is close enough to desired goal

xbox = 0
##-----------------------------INIT---------------------------------##



##--------------------------Global Path--------------------------##
def addGoal(msg):
	global globalGoalList
	globalGoalList.append(msg)
		

def sendGoalToAstar():
	global globalGoalList
	global currentPosition
	global the_path
	
	nextGoal = PoseStamped()

	if len(globalGoalList) > 0:
		nextGoal = globalGoalList.pop(0)
		RvizCallback(nextGoal)

def poseCloseEnoughToPathGoal(): #returns true if close enough to the goal
	global pathGoal
	global currentPosition
	global planCloseEnough
	
	distance = np.sqrt(np.power(currentPosition.linear.x - pathGoal.linear.x,2)+np.power(currentPosition.linear.y - pathGoal.linear.y,2))
	
	if (distance < planCloseEnough): #and (angle < 0.6):
		#print('FAKKING CLAS ENAFF--------------------------')
		return True
	else:
		return False


##--------------------------Global Path--------------------------##


class Node:
	def __init__(self,x=0,y=0,z=0,theta=0,parent=None,heuristic=5,cost_to_come=0.4, turnTheta = 0):
		self.x = x
		self.y = y
		self.z = z
		self.theta = theta
		self.parent = parent
		self.heuristic = heuristic
		self.cost_to_come = cost_to_come
		self.cost = self.heuristic + self.cost_to_come
		self.representation = [self.x, self.y, self.theta]
		#self.turnTheta = turnTheta

	def update_cost(self):
		self.cost = self.heuristic + self.cost_to_come


def update_marker(msg):
	global marker_array
	marker_array = msg
	update_obstacle_list()


def getCostFromParents(node):
	cost = np.sqrt(np.power(node.parent.x - node.x,2) + np.power(node.parent.y - node.y,2))
	while node.parent:
		cost += node.parent.cost_to_come
		node = node.parent
	return cost

def getPosition(msg):

	global currentPosition
	currentPosition.linear.x = msg.pose.position.x
	currentPosition.linear.y = msg.pose.position.y
	orientation_q = msg.pose.orientation #orientation expressed in quaternions
	orientation_q_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w] #store the quaternions in an array

	(roll, pitch, yaw) = euler_from_quaternion(orientation_q_list) #translates from quaternion to euler angles
	currentPosition.angular.z = yaw
	#print(currentPosition)

def RvizCallback(msg):
	global goal

	orientation_q = msg.pose.orientation #orientation expressed in quaternions
	orientation_q_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w] #store the quaternions in an array

	(roll, pitch, yaw) = euler_from_quaternion(orientation_q_list) #translates from quaternion to euler angles
	#"forward" is the x direction so negative yaw means a clockwise rotation
	#return/publish the pose expressed as Twist
	goal.linear.x = msg.pose.position.x
	goal.linear.y = msg.pose.position.y
	goal.linear.z = 0#msg.pose.position.z

	goal.angular.x = roll
	goal.angular.y = pitch
	goal.angular.z = yaw
	clearAll()
	A_star()
	

def clearAll():
	global black_list
	black_list = []
	
	global Q
	global the_path
	the_path = Path()
	the_path.header.frame_id = 'map'

	global currentPosition
	Q = []

	origin = Node(currentPosition.linear.x, currentPosition.linear.y, currentPosition.linear.z, currentPosition.angular.z) #start

	Q.append(origin)
	
def calculate_heuristic(x,y):
	global goal
	return np.sqrt(np.power(goal.linear.x-x,2)+np.power(goal.linear.y-y,2))

def update_obstacle_list():
	global marker_array
	global obstacle_list
	obstacle_list = []
	for point in marker_array.markers[16].points:

		obstacle_list.append(point)

		
def generate_nodes(parent):
	global planStep
	global planAngle
	global black_list
	global Q
	#global i
	black_list.append(parent) #To remember which nodes have been evaluated
	L = planAngle#vinklar 
	step = planStep
	for angle in L:
		x_temp = np.cos(parent.theta + angle)*step
		y_temp = np.sin(parent.theta + angle)*step
		xn = x_temp + parent.x
		yn = y_temp + parent.y
		thetan = parent.theta+angle
		#if child is suitable to be created (not already visited, not in obstacle etc)
		if is_node_ok(xn, yn, thetan):#, angle, parent.turnTheta):
			#print("node is ok")
			child = Node(xn,yn,0,thetan,parent)
			child.heuristic = calculate_heuristic(xn,yn)
			#child.cost_to_come = getCostFromParents(child)
			child.update_cost()
			#child.set_turnTheta(angle)
			#print("New node appended")
			Q.append(child)
		else:
			#print("node NOT okay")
			pass

def node_is_in_blacklist(x,y,theta):
	global black_list
	for node in black_list:
		distance = np.sqrt(np.square(x - node.x)+np.square(y - node.y))
		angle = abs(theta - node.theta)
		if (distance < 0.05 and angle < 0.1): #5cm and 0.1rad		
			return True
	return False

def get_path(node):
	if not node.parent == None:
		get_path(node.parent)
	
	global the_path
	
	pose = PoseStamped()
	pose.pose.position.x = node.x
	pose.pose.position.y = node.y
	pose.pose.position.z = node.z
	the_path.poses.append(pose)
		

def notNearObstacle(obstacle_list, x, y):
	#lagga till att kolla intersect?
	global max_obstacle_height
	global min_obstacle_height
	global min_obstacle_distance
	for obstacle in obstacle_list:
		if (obstacle.z > min_obstacle_height) and (obstacle.z < max_obstacle_height):
			distance = np.sqrt(np.square(x - obstacle.x)+np.square(y - obstacle.y))
			
			if distance < min_obstacle_distance:			
				return False
	return True	
			

def is_node_ok(x, y, theta):#, turnTheta, parentTurnTheta): #checks if node is suitable to be created
	global obstacle_list
	
	if node_is_in_blacklist(x,y,theta):
		return False



	#May be consider orientation
	return notNearObstacle(obstacle_list, x, y)


def getBestNode():
	global Q
	bestIndex = 0
	currentBest = Q[bestIndex]
	for i in range(len(Q)):
		if Q[i].cost < currentBest.cost:
			currentBest = Q[i]
			bestIndex = i
	bestNode = Q.pop(bestIndex)
	return bestNode

def closeEnoughTogoal(node): #returns true if close enough to the goal
	global goal
	global poseCloseEnoughToGoal
	global angleCloseEnoughToGoal

	distance = np.sqrt(np.power(node.x - goal.linear.x,2)+np.power(node.y - goal.linear.y,2))
	angle = abs(node.theta - goal.angular.z)
	if (distance < poseCloseEnoughToGoal) and (angle < angleCloseEnoughToGoal):
		print('Close enough')
		return True
	else:
		return False


def A_star():
	global Q
	global black_list
	path_ok = Int8()
	global the_path
	global pub_path
	global path_is_ok
	global A_star_iterations
	i = 0
	global obstacle_list
	#global start
	global pathGoal
	while Q:
		#get bestnode
		bestNode = getBestNode()
		if i < A_star_iterations:
			i = i + 1
			print("A* iteration: " + str(i))

			
			if closeEnoughTogoal(bestNode):
				#return success and the path
				print('Close to goal')

				get_path(bestNode)
				pathGoal.linear.x = bestNode.x
				pathGoal.linear.y = bestNode.y
				pub_path.publish(the_path)
				break
			else:
				generate_nodes(bestNode)
		else:
			print('Non-optimal path')
			get_path(bestNode)
			pathGoal.linear.x = bestNode.x
			pathGoal.linear.y = bestNode.y
			pub_path.publish(the_path)
			break
def xboxTakeover(msg):
	global xbox
	global globalGoalList
	global currentPosition
	nextGoal = PoseStamped()
	xbox = msg.data
	print(xbox)
	#if xbox == 0:
	globalGoalList = []
	nextGoal.pose.position.x = currentPosition.linear.x
	nextGoal.pose.position.y = currentPosition.linear.y
	nextGoal.pose.position.z = currentPosition.linear.z
	RvizCallback(nextGoal) #planerar ju fran currentPos till nextgoal




def main():
	global black_list
	global Q
	global obstacle_list
	global xbox
	
	rate = rospy.Rate(10)

	sub_markerArray_map = rospy.Subscriber('occupied_cells_vis_array', MarkerArray, update_marker) #subscribes to the marker array which contains detected physical points in space
	origin = Node()
	Q.append(origin)
	subRvizGoal = rospy.Subscriber('next_goal_global_path', PoseStamped, RvizCallback)
	subRvizGoal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, addGoal)
	subPose = rospy.Subscriber('/zed/zed_node/pose', PoseStamped, getPosition)
	sub_xboxTakeover = rospy.Subscriber('xbox_takeover', Int64, xboxTakeover)
	while not rospy.is_shutdown():
		if (poseCloseEnoughToPathGoal() == True and xbox == 0): #sends next goal to Astar if close enough to path goal
			sendGoalToAstar()		
		rate.sleep()
		#show_occupied_points()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
