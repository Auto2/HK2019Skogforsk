#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

##-----------------------------DESCRIPTION---------------------------------##
#Transforms a coordinate in the map frame to _____
#
#Uses the measured pose from sensors and some trigonometry to achieve this
#
#Must receive the robot pose in the map as a Twist message as well as a goal
#coordinate given in the map frame (Point)
##-----------------------------DESCRIPTION---------------------------------##


##-----------------------------INIT---------------------------------##
rospy.init_node('coord_transform', anonymous=True)
pub_goal = rospy.Publisher('goal_rig', Point, queue_size = 1) #goal coordinates relative to rig frame
rosRate = 10
rate = rospy.Rate(rosRate)

pose = Twist() #testrig pose in map
goal_map = Point() #goal rel map

#goal_map.x = 0.0 #pick values for debugging or comment out
#goal_map.y = 0.0 #pick values for debugging or comment out

goal = Point() #goal rel testrig
##-----------------------------INIT---------------------------------##

def set_pose(msg):
    global pose
    pose = msg

def set_goal_map(msg):
    global goal_map
    goal_map = msg

def coord_transform(): #use the measured pose and goal in the map to
    global pose
    global goal_map
    global goal

    #dx = xm-xr  distance from robot in unrotated coord.sys
    dx = goal_map.x - pose.linear.x
    #dy = ym-yr  distance from robot in unrotated coord.sys
    dy = goal_map.y - pose.linear.y

    #rotation of carteesian coordinate systems based on yaw https://en.wikipedia.org/wiki/Rotation_of_axes
    #[xr] = [cos  sin][x]
    #[yr]   [-sin cos][y]
    #distances and angle to goal rel testrig
    xr = np.cos(pose.angular.z)*dx + np.sin(pose.angular.z)*dy
    yr = (-1)*np.sin(pose.angular.z)*dx + np.cos(pose.angular.z)*dy

    #store as goal rel testrig
    goal.x = xr
    goal.y = yr

    pub_goal.publish(goal) #publishes the goal relative to the testrig


def main():
    subpose = rospy.Subscriber('zed/zed_node/pose_twist', Twist, set_pose) #estimated rig pose from SLAM
    subgoal = rospy.Subscriber('goal_map', Point, set_goal_map) #goal in map frame
    while not rospy.is_shutdown():
        coord_transform()
        rate.sleep()

if __name__ == '__main__':

    try:
        main()
    except rospy.ROSInterruptException:
        pass
