
#!/usr/bin/env python

import numpy as np
import rospy

from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped, Twist	
from tf.transformations import euler_from_quaternion 

##-----------------------------DESCRIPTION---------------------------------##
#
##-----------------------------DESCRIPTION---------------------------------##


##-----------------------------INIT---------------------------------##
rospy.init_node('astar', anonymous=True)
pub_goal = rospy.Publisher('occupied_coordinates_in_map', Point, queue_size = 1) #goal coordinates relative to rig frame 

marker_array = MarkerArray()
obstacle_list = [] #list of occupied points

pub_path = rospy.Publisher('Astar_path',Path,queue_size=1)
path_is_ok = rospy.Publisher('path_is_ok',Int8,queue_size=1)  

rate = rospy.Rate(10)

Q=[]
black_list=[]
the_path = Path() 
globalGoalList = [] # newline

the_path.header.frame_id = 'map'
goal = Twist()
pathGoal = Twist()  # newline

#goal.x = -8
#goal.y = -5

currentPosition = Twist()
#currentTheta = 0
max_obstacle_height = 5.0
min_obstacle_height = 0.2
min_obstacle_distance = 0.5
#i = 0 							
initialPoint = 1 #newline


						#JOCKE			

##-----------------------------INIT---------------------------------##



##--------------------------Global Path--------------------------##
def addGoal(msg):
	global globalGoalList
	global initialPoint
	globalGoalList.append(msg)
	#tmpGoal = globalGoalList.pop(0)
	#pubGlobalPathNext.publish(tmpGoal)
	#if (len(globalGoalList) == 1) and (initialPoint == 1):#publish immediately if it's the initial point	
	#	initialPoint = 0
	#	tmpGoal = globalGoalList.pop(0)
	#	RvizCallback(tmpGoal)
	#else:
	#	tmpGoal = globalGoalList.pop(0)
	#	RvizCallback(tmpGoal)
	#tmpGoal = globalGoalList.pop(0)
	#RvizCallback(tmpGoal)
		

def sendGoalToAstar():
	global globalGoalList
	if len(globalGoalList) > 0:
		nextGoal = globalGoalList.pop(0)
		RvizCallback(nextGoal)

def poseCloseEnoughToPathGoal(): #returns true if close enough to the goal
	global pathGoal
	global currentPosition
	
	distance = np.sqrt(np.power(currentPosition.linear.x - pathGoal.linear.x,2)+np.power(currentPosition.linear.y - pathGoal.linear.y,2))
	angle = np.abs(pathGoal.angular.z - currentPosition.angular.z)
	
	#print('DISTANCE _______________________________')
	#print(distance)
	#print('------------------------------------------------')
	
	#angle = abs(currentPosition.z - pathGoal.angular.z)
	#print(distance)
	if (distance < 0.3) and (angle < 0.3):
		#print('FAKKING CLAS ENAFF--------------------------')
		return True
	else:
		return False


##--------------------------Global Path--------------------------##

#jocke not between

class Node:
	def __init__(self,x=0,y=0,z=0,theta=0,parent=None,heuristic=5,cost_to_come=0.1):
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
#start = Node() # inold


def update_marker(msg):
	global marker_array
	marker_array = msg
	update_obstacle_list()
	#print(len(obstacle_list)) 

def getCostFromParents(node):
	cost = np.sqrt(np.power(node.parent.x - node.x,2) + np.power(node.parent.y - node.y,2))
	while node.parent:
		cost += node.parent.cost_to_come
		node = node.parent
	return cost

def getPosition(msg):
	#global currentX
	#global currentY
	#global currentTheta
	global currentPosition
	currentPosition.linear.x = msg.pose.pose.position.x
	currentPosition.linear.y = msg.pose.pose.position.y
	orientation_q = msg.pose.pose.orientation #orientation expressed in quaternions
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
	#goal = Twist()

	goal.linear.x = msg.pose.position.x
	goal.linear.y = msg.pose.position.y
	goal.linear.z = 0#msg.pose.position.z

	goal.angular.x = roll
	goal.angular.y = pitch
	goal.angular.z = yaw
	clearAll()
	A_star()   
	

def clearAll():
	global Q
	global the_path
	the_path = Path()		  # newline
	the_path.header.frame_id = 'map'  # newline

	global currentPosition
	Q = []
	#start.parent = None
	#start.heuristic = 5
	#start.cost_to_come = 0.1
	origin = Node(currentPosition.linear.x, currentPosition.linear.y, currentPosition.linear.z, currentPosition.angular.z) #start
	#print("CLEAAAAAAAAAAARING ALLLLLLLLLLLLLLLLLLLLLLLL ")
	#print(the_path)
	#print("APPEND ORIGIN")
	Q.append(origin)  
	
def calculate_heuristic(x,y):
	global goal
	return np.sqrt(np.power(goal.linear.x-x,2)+np.power(goal.linear.y-y,2))

def update_obstacle_list():
	global marker_array
	global obstacle_list
	obstacle_list = []
	for point in marker_array.markers[16].points:
		#print(point.points[0])
		obstacle_list.append(point)
		
def generate_nodes(parent):
	
	global black_list
	global Q
	global i
	black_list.append(parent) #To remember which nodes have been evaluated
	L = [-0.125, -0.6125 ,0, 0.6125, 0.125] #angels 
	step = 0.25
	for angle in L:
		x_temp = np.cos(parent.theta + angle)*step
		y_temp = np.sin(parent.theta + angle)*step
		xn = x_temp + parent.x
		yn = y_temp + parent.y
		thetan = parent.theta+angle
		#if child is suitable to be created (not already visited, not in obstacle etc)
		if is_node_ok(xn, yn, thetan):
			#print("node is ok")
			child = Node(xn,yn,0,thetan,parent)
			child.heuristic = calculate_heuristic(xn,yn)
			#child.cost_to_come = getCostFromParents(child)
			child.update_cost()
			#print("New node appended")
			Q.append(child)
		else:
			print("node NOT okay")
			pass



def node_is_in_blacklist(x,y,theta):
	global black_list
	for node in black_list:
		distance = np.sqrt(np.square(x - node.x)+np.square(y - node.y))
		angle = abs(theta - node.theta)
		if (distance < 0.05) and (angle < 0.1): #5cm and 0.1rad	
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
		

def notTooCloseToObstacle(obstacle_list, x, y):
	global max_obstacle_height
	global min_obstacle_height
	global min_obstacle_distance
	for obstacle in obstacle_list:
		if (obstacle.z > min_obstacle_height) and (obstacle.z < max_obstacle_height):
			distance = np.sqrt(np.square(x - obstacle.x)+np.square(y - obstacle.y))
			
			if distance < min_obstacle_distance:			
				return False
	return True	

			

def is_node_ok(x, y, theta): #checks if node is suitable to be created
	global obstacle_list
	
	if node_is_in_blacklist(x,y,theta):
		return False

	#May be consider orientation
	return notTooCloseToObstacle(obstacle_list, x, y)


def getBestNode():
	global Q
	bestIndex = 0
	currentBest = Q[bestIndex]
	for i in range(len(Q)):
		if Q[i].cost < currentBest.cost:
			currentBest = Q[i]
			bestIndex = i
	bestNode = Q.pop(bestIndex)
	#print(bestNode.representation)
	return bestNode

def closeEnoughTogoal(node): #returns true if close enough to the goal
	global goal
	distance = np.sqrt(np.power(node.x - goal.linear.x,2)+np.power(node.y - goal.linear.y,2))
	angle = abs(node.theta - goal.angular.z)
	#print(distance)
	if (distance < 0.15) and (angle < 0.15):
		print('Close enough')
		return True
	else:
		return False

#jocke
	

def A_star():
	global Q
	global black_list
	path_ok = Int8()
	global the_path
	global pub_path
	global path_is_ok
	i = 0
	global obstacle_list
	#global start
	global pathGoal
	while Q:
		#get bestnode
		bestNode = getBestNode()
		if i < 250:
			i = i + 1
			#print("Iteration: " + str(i))
			#print("Obstacle list size: " + str(len(obstacle_list)))
			#print("Node coordinates (x,y,theta): ")
			#print(bestNode.representation)
			#check if goal
			
			if closeEnoughTogoal(bestNode):
				#return success and the path
				print('Close to goal')
				#path_ok_value = 1
				#path_ok.data = path_ok_value
				#path_is_ok.publish(path_ok)
				get_path(bestNode)
				pathGoal.linear.x = bestNode.x  #newline
				pathGoal.linear.y = bestNode.y  #newline
				pub_path.publish(the_path)
				break
			else:
				generate_nodes(bestNode)
		else:
			#print('Non-optimal path')
			get_path(bestNode)
			pathGoal.linear.x = bestNode.x  #newline
			pathGoal.linear.y = bestNode.y  #newline
			pub_path.publish(the_path)
			
			break

def main():
	global black_list
	global Q
	global obstacle_list

	sub_markerArray_map = rospy.Subscriber('occupied_cells_vis_array', MarkerArray, update_marker) #subscribes to the marker array which contains detected physical points in space
	origin = Node()
	Q.append(origin)
	subRvizGoal = rospy.Subscriber('next_goal_global_path', PoseStamped, RvizCallback)
	subRvizGoal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, addGoal)
	subPose = rospy.Subscriber('/my_odom', Odometry, getPosition)
	while not rospy.is_shutdown():
		if poseCloseEnoughToPathGoal() == True: #sends next goal to Astar if close enough to path goal #newline
			sendGoalToAstar()
		rate.sleep()
		#show_occupied_points()

if __name__ == '__main__':

	try:
		main()
	except rospy.ROSInterruptException:
		pass
