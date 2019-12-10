#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion



##-----------------------------INIT---------------------------------##
rospy.init_node('quaternion_to_euler', anonymous=True)
pubPose = rospy.Publisher('odom_twist', Twist, queue_size = 1)
rate = rospy.Rate(10)
##-----------------------------INIT---------------------------------##


def odom_to_twist(msg): #translates quaternion angles to euler angles (radians)
    orientation_q = msg.pose.pose.orientation #orientation expressed in quaternions
    orientation_q_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w] #store the quaternions in an array

    (roll, pitch, yaw) = euler_from_quaternion(orientation_q_list) #translates from quaternion to euler angles
    #"forward" is the x direction so negative yaw means a clockwise rotation
    #return/publish the pose expressed as Twist
    twist_pose = Twist()

    twist_pose.linear.x = msg.pose.pose.position.x
    twist_pose.linear.y = msg.pose.pose.position.y
    twist_pose.linear.z = msg.pose.pose.position.z

    twist_pose.angular.x = roll
    twist_pose.angular.y = pitch
    twist_pose.angular.z = yaw

    pubPose.publish(twist_pose)


def main():

	sub_zed = rospy.Subscriber('my_odom', Odometry, odom_to_twist)
	while not rospy.is_shutdown():
		rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


# NOT IN USE!?!?!?