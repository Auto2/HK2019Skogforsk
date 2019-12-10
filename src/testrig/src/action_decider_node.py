#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path


##-----------------------------DESCRIPTION---------------------------------##
#Plans action.....
#
#Subscribes to "goal_rig": this is a point which represents the goal in the robot frame
#[x, y, z] = [x, y, NULL]
#
#
#Publishes "motor_action_sw": this message is a number which the motor and
#waist controllers can interpret and drive accordingly
##-----------------------------DESCRIPTION---------------------------------##


##-----------------------------INIT---------------------------------##
rospy.init_node('action_decider')
pubAction = rospy.Publisher('motor_action', Int64, queue_size = 1)
pubPWM = rospy.Publisher('motor_pwm',Int64,queue_size=1)
pubReachedGoal = rospy.Publisher('goal_reached',Int64,queue_size=1)
pub_rig_angle = rospy.Publisher('rig/angle_to_goal',Float64,queue_size=1)
rate = rospy.Rate(10) #var 10 innan
rospy.loginfo("Initiated: action_decider_node.py")

goal = Point()#[0, 0] #init goal in origin [x, y]
pose = Twist()
action = 4
oldAction = 0
#goal.x = 1
#goal.y = 0.1

xbox = 0

#For tests
x_pos = []
y_pos = []
x_path = []
y_path = []

closeEnough = 0.3 #distance from goal when it is deemed that the testrig is close enough to have arrived (meters)
angleDev = 0.01 #angle deviation to decide when it is time to turn (radians)
reverseDist = 0.5 #minimum distance (backwards or sideways -45 - 45 deg) where testrig starts reversing
##-----------------------------INIT---------------------------------##

def get_goal(msg):
    global goal
    goal = msg

def get_pose(msg): #get roation of the rig
    global pose
    pose = msg

def save_pose(msg):
    global x_pos
    global y_pos
    x_pos.append(msg.pose.position.x)
    y_pos.append(msg.pose.position.y)

def save_path(msg):
    global x_path
    global y_path
    for msgpose in msg.poses:
	x_path.append(msgpose.pose.position.x)
	y_path.append(msgpose.pose.position.y)

def xboxTakeover(msg):
    global xbox, x_pos, y_pos, x_path, y_path
    xbox = msg.data
    if xbox == 1:
	print('x_pos = ', x_pos)
	print('y_pos = ', y_pos)
	print('x_path = ', x_path)
	print('y_path = ', y_path)
	x_pos = []
	x_pos = []
	x_path = []
	y_path = []

def calculateAngleAndDistance():
    global goal
    global pose

    xr = goal.x #x value for goal relative to the robot frame, positive is forward
    yr = goal.y #y value for goal relative to the robot frame, positive is to the right

    goal_angle = np.arctan2(yr,xr) #- pose.angular.z
    distance = np.sqrt(np.square(xr) + np.square(yr))
    rig_angle = Float64()
    rig_angle.data = goal_angle
    pub_rig_angle.publish(rig_angle)
    return distance, goal_angle

def decideOnAction():
    global closeEnough
    global angleDev
    global xbox
    
    if (xbox == 0):
        pwm_msg = Int64()
        pwm_msg.data = 30
        pubPWM.publish(pwm_msg)
    action = 4
    
    distance, angle = calculateAngleAndDistance()
    
    if (xbox == 0) :
        if ((angle > 3 or angle < -3) and distance < reverseDist and distance >= closeEnough): ## if not reached goal and point not within acceptable angles, try backing
	    action = 1 
        elif angle <= angleDev and angle >= -angleDev and distance >= closeEnough: ##if kinda infront and at least 'closeEnough' meters to the point
            action = 7
        elif angle > angleDev and distance >= closeEnough: ##if to the left and at least selected distance to the point
            action = 6
        elif angle < -angleDev and distance >= closeEnough: ##if to the right and at least selected distance to the point
            action = 8
        else:
            action = 4 #stand still if no conditions are fulfilled
    return action


def doStuff():
	global oldAction
	global action
	oldAction = action
	action = decideOnAction() #decides what action to take based on where the current goal is relative to the testrig
	if not oldAction == 4 and not action == 4:
	    action_msg = Int64()
            action_msg.data = action
	    pubAction.publish(action_msg) #publish the action
	reachedGoal = Int64()
	if action == 4:
	    #rospy.loginfo("Reached checkpoint!")
      	    #rospy.sleep(4.0)
            reachedGoal.data = 1
	    pubReachedGoal.publish(reachedGoal)
	else:
	    reachedGoal.data = 0
	    pubReachedGoal.publish(reachedGoal)	
	

def main():
	pwm_msg = Int64()
	pwm_msg.data = 30
	pubPWM.publish(pwm_msg)
	subGoal = rospy.Subscriber('goal_rig', Point, get_goal)
        sub_pose = rospy.Subscriber('zed/zed_node/pose_twist', Twist, get_pose)
	sub_xboxTakeover = rospy.Subscriber('xbox_takeover', Int64, xboxTakeover)
	sub_pose_for_test= rospy.Subscriber('zed/zed_node/pose', PoseStamped, save_pose)
	sub_path_for_test= rospy.Subscriber('Astar_path', Path, save_path)
		
	reachedGoal = Int64()
	reachedGoal.data = 1
	pubReachedGoal.publish(reachedGoal)
	while not rospy.is_shutdown():
		doStuff()
		rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
