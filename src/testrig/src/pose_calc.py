#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

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
pubPose = rospy.Publisher('pose', Twist, queue_size = 1)
rosRate = 10
rate = rospy.Rate(rosRate)

pose = Twist()
zedPose = PoseStamped()
speed = Float64()
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
MAX_TURN_RADIUS = 100000 #mm
MIN_TURN_RADIUS = 600 #mm
turnRate = 10 #mm/increment
##--------------------------------------------------------##


def setPoseCallback(setPose):
    global pose
    pose = setPose

def zedPoseCallback(setPose):
    global zedPose
    zedPose = setPose

def setSpeedCallback(spd):
    global speed
    speed = spd
    
def setTurnRadiusCallback(radius):
    global turn_radius
    turn_radius = radius

def updatePose():
    global pose, zedPose, speed, turn_radius

    #update pose according to the speed and turning radius over the dt given by rosRate
    dt = float(float(1)/rosRate)
    if turn_radius.data < 10 and turn_radius.data > -10:
        theta = 0
    else:
        theta = round((180/(np.pi)) * (round(speed.data, 3)*1000*dt)/turn_radius.data, 2)

    dx = round((turn_radius.data) * (1.0 - np.cos(theta)), 3) /1000
    dy = round((turn_radius.data) * np.sin(theta), 3) /1000
    dz = 0

    #if speed.data <= 0:
        #dx = (-1)*dx
	#dy = (-1)*abs(dy)
    if turn_radius.data > 0:
	dx = abs(dx)
    elif turn_radius.data < 0:
        dx = (-1)*abs(dx)

    [droll, dpitch, dyaw] = [0, 0, -theta]
    updatedPose = Twist()

    #transformation to world-relative changes
    #x' = [cos, -sin]|x|
    #y'   [sin,  cos]|y|

    dxm = np.cos(pose.angular.z)*dx - np.sin(pose.angular.z)*dy
    dym = np.sin(pose.angular.z)*dx + np.cos(pose.angular.z)*dy

    updatedPose.linear.x = zedPose.pose.position.x#pose.linear.x + dxm
    updatedPose.linear.y = zedPose.pose.position.y#pose.linear.y + dym
    updatedPose.linear.z = zedPose.pose.position.z#pose.linear.z + dz

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

    subRadius = rospy.Subscriber('desired_radius', Int64, setTurnRadiusCallback)
    subSpeed = rospy.Subscriber('speed', Float64, setSpeedCallback)
    subpose = rospy.Subscriber('pose', Twist, setPoseCallback)
    subposezed = rospy.Subscriber('/zed/zed_node/pose', PoseStamped, zedPoseCallback)
    while not rospy.is_shutdown():
        doStuff()
        rate.sleep()

if __name__ == '__main__':

    try:
        main()
    except rospy.ROSInterruptException:
        pass
