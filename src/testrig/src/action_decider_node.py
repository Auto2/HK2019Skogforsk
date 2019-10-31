#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Int64
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

##-----------------------------DESCRIPTION---------------------------------##
#Plans action..... 
#
#Subscribes to "cmd_goal": this is a point which represents the goal
#[x, y, z] = [x, y, NULL]
#
#Subscribes to "pose": this is a Twist message  which represents the pose
#of the testrig
#
#
#Publishes "motor_action_sw": this message is a number which the motor and
#waist controllers can interpret and drive accordingly
##-----------------------------DESCRIPTION---------------------------------##


##-----------------------------INIT---------------------------------##
rospy.init_node('action_decider')
pubAction = rospy.Publisher('motor_action_sw', Int64, queue_size = 2)
rate = rospy.Rate(10)

goal = Point()#[0, 0] #init goal in origin [x, y]
goal.x = 0 #x
goal.y = 0 #y

pose = Twist()
pose.linear.x = 0 #x
pose.linear.y = 0 #y
pose.angular.z = 0 #yaw
##-----------------------------INIT---------------------------------##


##----------------CONFIGURATION PARAMETERS----------------##
#all measurements in mm and seen from top plane   
L = 500 #distance between the two waists
d_f = 235 #distance between front waist and front wheel axle
d_m = 285 #distance between front waist and middle wheel axle
d_r = 235 #distance between front waist and rear wheel axle
allowedTwist = [0, 45] #maximum twist of waists
tolerance = 0.01 #percentage based error tolerance for acceptable turn radius
iterationLimit = 1500 #maximum amount of iterations
MAX_TURN_RADIUS = 4006 #mm
MIN_TURN_RADIUS = 500 #mm
turnRate = 10 #mm/increment
##--------------------------------------------------------##
def setGoalCallback(point):
    global goal
    goal = point

def setPoseCallback(setPose):
    global pose
    pose = setPose

def calculateAngleAndDistance():
    global goal, pose

    dx = goal.x - pose.linear.x
    dy = goal.y - pose.linear.y
    xr = np.cos(pose.angular.z) * dx + np.sin(pose.angular.z) * dy #x value for goal relative to the robot frame 
    yr = (-1)*np.sin(pose.angular.z) * dx + np.cos(pose.angular.z) * dy #y value for goal relative to the robot frame 

    goal_angle = np.arctan2(yr,xr)
    distance = np.sqrt(np.square(xr) + np.square(yr))

    return distance, goal_angle

def decideOnAction():
    distance, angle = calculateAngleAndDistance()
    if angle <= 0.1 and angle >= -0.1 and distance >= 50: ##if kinda infront and at least 50 mm to the point
        action = 7
    elif angle > 0.1 and distance >= 50: ##if to the left and at least 50 mm to the point
        action = 6
    elif angle < -0.1 and distance >= 50: ##if to the right and at least 50 mm to the point
        action = 8
    else:
        action = 4 #stand still if no conditions are fulfilled

    return action


def doStuff():
        action = decideOnAction() #
        action_msg = Int64()
        action_msg.data = action
        pubAction.publish(action_msg)


def main():

        subGoal = rospy.Subscriber('cmd_goal', Point, setGoalCallback)
        subpose = rospy.Subscriber('pose', Twist, setPoseCallback)
        while not rospy.is_shutdown():
                doStuff()
                rate.sleep()

if __name__ == '__main__':

    try:
        main()
    except rospy.ROSInterruptException:
        pass

