#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Int64
from geometry_msgs.msg import Twist

##-----------------------------DESCRIPTION---------------------------------##
#Iterates the pose of the testrig.
#
#Subscribes to "turn_radius": this is a Twist msg which represents the current
#turn radius
#
#Subscribes to "speed": this is an In64 msg which represents the current
#speed of the testrig
#
#Subscribes to "pose": this is a Twist msg which represents the current pose
#of the testrig
#
#
#Publishes "pose": this is a Twist msg which represents the current pose
#of the testrig
#
#2019/10
#Alexander Axelsson
##-----------------------------DESCRIPTION---------------------------------##


##-----------------------------INIT---------------------------------##
rospy.init_node('pose_calc')
pubPose = rospy.Publisher('pose', Twist, queue_size = 2)
rosRate = 10
rate = rospy.Rate(rosRate)

pose = Twist()
speed = Int64()
turn_radius = Int64()
##-----------------------------INIT---------------------------------##


##----------------CONFIGURATION PARAMETERS----------------##
#all measurements in mm and seen from top plane
#NOT IN USE AT THE MOMENT
L = 500 #distance between the two waists
d_f = 235 #distance between front waist and front wheel axle
d_m = 285 #distance between front waist and middle wheel axle
d_r = 235 #distance between front waist and rear wheel axle
allowedTwist = [0, 45] #maximum twist of waists
tolerance = 0.01 #percentage based error tolerance for acceptable turn radius
iterationLimit = 1500 #maximum amount of iterations
MAX_TURN_RADIUS = 4000 #mm
MIN_TURN_RADIUS = 500 #mm
turnRate = 10 #mm/increment
##--------------------------------------------------------##


def setPoseCallback(setPose):
    global pose
    pose = setPose

def setSpeedCallback(spd):
    global speed
    speed = spd

def setTurnRadiusCallback(radius):
    global turn_radius
    turn_radius = radius

def updatePose():
    global pose, speed, turn_radius

    #update pose according to the speed and turning radius over the dt given by rosRate
    dt = 1/rosRate
    if turn_radius.data < 50 and turn_radius.data > -50:
        theta = 0
    else:
        theta = (180/(np.pi)) * (speed.data*dt)/turn_radius.data

    dx = turn_radius.data * (1 - np.cos(theta))
    dy = turn_radius.data * np.sin(theta)
    dz = 0

    [droll, dpitch, dyaw] = [0, 0, -theta]
    updatedPose = Twist()

    updatedPose.linear.x = pose.linear.x + dx
    updatedPose.linear.y = pose.linear.y + dy
    updatedPose.linear.z = pose.linear.z + dz

    updatedPose.angular.x = pose.angular.x + droll
    updatedPose.angular.y = pose.angular.y + dpitch
    updatedPose.angular.z = pose.angular.z + dyaw

    return updatedPose


def doStuff():
        updatedPose = updatePose() #
        #action_msg = Int64()
    #action_msg.data = action
        pubPose.publish(updatedPose)


def main():

    subRadius = rospy.Subscriber('actual_radius', Int64, setTurnRadiusCallback)
    subSpeed = rospy.Subscriber('speed', Int64, setSpeedCallback)
    subpose = rospy.Subscriber('pose', Twist, setPoseCallback)

