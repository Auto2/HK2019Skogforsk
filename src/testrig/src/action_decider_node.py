#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist


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
#goal.x = 1
#goal.y = 0.1

closeEnough = 0.25 #distance from goal when it is deemed that the testrig is close enough to have arrived (meters)
angleDev = 0.7 #angle deviation to decide when it is time to turn (radians)
##-----------------------------INIT---------------------------------##

def get_goal(msg):
    global goal
    goal = msg

def get_pose(msg): #get roation of the rig
    global pose
    pose = msg


def calculateAngleAndDistance():
    global goal
    global pose

    xr = goal.x #x value for goal relative to the robot frame, positive is forward
    yr = goal.y #y value for goal relative to the robot frame, positive is to the right

    goal_angle = np.arctan2(yr,xr) #+ pose.angular.z
    distance = np.sqrt(np.square(xr) + np.square(yr))
    rig_angle = Float64()
    rig_angle.data = goal_angle
    pub_rig_angle.publish(rig_angle)

    return distance, goal_angle


def decideOnAction():
    global closeEnough
    global angleDev
    
    pwm_msg = Int64()
    pwm_msg.data = 30
    pubPWM.publish(pwm_msg)
    
    distance, angle = calculateAngleAndDistance()
   
    if angle <= angleDev and angle >= -angleDev and distance >= closeEnough: ##if kinda infront and at least 50 mm to the point
        action = 7
    elif angle > angleDev and distance >= closeEnough: ##if to the left and at least selected distance to the point
        action = 6
    elif angle < -angleDev and distance >= closeEnough: ##if to the right and at least selected distance to the point
        action = 8
    else:
        action = 4 #stand still if no conditions are fulfilled

    	#reachedGoal = Int64()
    #if distance < closeEnough: #reached current goal
	#print("Reached goal!")
	#rospy.sleep(4.0)
	#reachedGoal.data = 1
	#pubReachedGoal.publish(reachedGoal)
    #else:
    #    reachedGoal.data = 0
    #    pubReachedGoal.publish(reachedGoal)

    return action


def doStuff():
	action = decideOnAction() #decides what action to take based on where the current goal is relative to the testrig
	action_msg = Int64()
        action_msg.data = action
	pubAction.publish(action_msg) #publish the action
	reachedGoal = Int64()
	if action == 4:
	    rospy.loginfo("Reached checkpoint!")
	    rospy.sleep(4.0)
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
	while not rospy.is_shutdown():
		doStuff()
		rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
