#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64
from std_msgs.msg import String
from geometry_msgs.msg import Point
import time
from sensor_msgs.msg import Joy

# --------------------------------------------------------------
class Interpreter:
    def __init__(self):
        # init the interpreter object
        # ------------------------------------------------------
        #      Init some ROS stuff
        self.pub1 = rospy.Publisher('motor_action',Int64,queue_size=1)
        self.pub2 = rospy.Publisher('motor_pwm',Int64,queue_size=1)

	self.pubWaist = rospy.Publisher('cmd_waist_twists',Point,queue_size=1)   
	#self.actual_angles = rospy.Subscriber('waist_angles',Twist,angleCallback)
	# ------------------------------------------------------
        #      Init some variables
        self.x = 0
        self.y = 0

	self.waist1 = 0
	self.waist2 = 0

        self.pwm = 10
        self.action = 4 # 4 is idle
	self.pwmButtonState = 0

    def interpret(self,joy):
        # A function that takes in the message from joy
        # -----------------------------------------------------
        # calculate PWM
        #neg_val = joy.axes[5] - 1 # ett tal mellan 0 och -2
        #pos_val = neg_val * (-1) # ett tal mellan 0 och 2
        #percent = pos_val/2 # ett tal mellan 0 och 1
        #p = 10 + percent*80
	if joy.buttons[0] == 1 and self.pwm < 90 and self.pwmButtonState == 0:
	    self.pwm = self.pwm + 10
	    self.publish_pwm()
	    self.pwmButtonState = 1
	elif joy.buttons[1] == 1 and self.pwm > 10 and self.pwmButtonState == 0:
	    self.pwm = self.pwm - 10
	    self.publish_pwm()
	    self.pwmButtonState = 1
	elif joy.buttons[0] == 0 and joy.buttons[1] == 0:
	    self.pwmButtonState = 0
	
        # -----------------------------------------------------
        #  transform PWM

        #p = p/10
        #p = round(p)
        #self.pwm = p*10

        # -----------------------------------------------------
        # Interpret motor action Left/right
        # Left = 1
        crossLR = joy.axes[0]
        crossUD = joy.axes[1]
	
	if (abs(crossUD)>0.1) :
	    if (abs(crossLR)>0.1) :
		self.waist1 = -45 * crossLR
		self.waist2 = self.waist1 * 1
	    else :
		self.waist1 = 0
		self.waist2 = 0
	    self.publishWaist() 

        #if crossLR < -0.1:
        #    self.x = self.x  1
        #elif crossLR > 0.1:
        #    self.x = -1
        #else :
        #    self.x = 0
        # -----------------------------------------------------
        # Interpret motor action up/down
        # Upp = 1
        if crossUD < -0.1:
            #self.y = -1
	    self.action = 1
        elif crossUD > 0.1:
            #self.y = 1
	    self.action = 7       
	else :
            #self.y = 0
	    self.action = 4        
	# -----------------------------------------------------
        # Calculate motor action and publish
        #self.action_recalc()
        self.publish()

    def action_recalc(self):
        # actions - imagine a keypad:
        # 6 7 8     FL F FR
        # 3 4 5  =  L  I  R
        # 0 1 2    BL B BR
        # F = forward, B = backward, L = left, R = right, I = idle
        self.action = self.x + 3*self.y + 4 # see numbers on "keypad" and directions above
	if self.action == 3:
            self.action = 4
        elif self.action == 5:
	    self.action = 4

    def publish(self):
        # action_msg is of type Int64
        action_msg = Int64()
        # pack action
        action_msg.data = self.action
        # pub
        self.pub1.publish(action_msg)
    
    def publishWaist(self):	
	waist_msg = Point()
	waist_msg.x = self.waist1
	waist_msg.y = self.waist2
	self.pubWaist.publish(waist_msg)

    def publish_pwm(self):
	pwm_msg = Int64()
	pwm_msg.data = self.pwm
	self.pub2.publish(pwm_msg)


def listener():
    rospy.init_node('interpreter_node',anonymous = True)
    tolk = Interpreter()
    sub1 = rospy.Subscriber('joy',Joy,tolk.interpret)
    rate = rospy.Rate(10) #10Hz
    while not rospy.is_shutdown():
	tolk.publish()
	rate.sleep()

if __name__ == '__main__':
    listener()
