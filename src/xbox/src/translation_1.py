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
	self.pubPend = rospy.Publisher('pendulum_action',Int64,queue_size=1)
	self.pubTakeoverGoal = rospy.Publisher('xbox_takeover',Int64,queue_size=1)
	
	#self.actual_angles = rospy.Subscriber('waist_angles',Twist,angleCallback)
	# ------------------------------------------------------
        #      Init some variables
        self.x = 0
        self.y = 0

	self.waist1 = 0
	self.waist2 = 0
	
	self.pendAction = 0

	self.pendSelect = 0
	self.pendButtonState = 0

	self.allpendstate = 0

        self.pwm = 10
        self.action = 4 # 4 is idle
	self.oldAction = 4
	self.pwmButtonState = 0

	self.takeover = 0
	self.takeoverButtonState = 0

    def interpret(self,joy):
        # A function that takes in the message from joy
        # -----------------------------------------------------
        # calculate PWM
        #neg_val = joy.axes[5] - 1 # ett tal mellan 0 och -2
        #pos_val = neg_val * (-1) # ett tal mellan 0 och 2
        #percent = pos_val/2 # ett tal mellan 0 och 1
        #p = 10 + percent*80
	self.oldAction = self.action
	
	# ---------- PWM --------------------------------------
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
	
	# ----------- TURN BOTH WAISTS ------------------------
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

	# ----------- TURN FRONT WAIST ------------------------
	frontLR = joy.axes[6]	# D-pad left/right
	
	if (abs(frontLR) > 0.1) :
	    if (frontLR > 0.1) :
	        self.waist1 = self.waist1 - 5
	    elif (frontLR < -0.1) :
	        self.waist1 = self.waist1 + 5
	    self.publishWaist()
	
	# -----------------------------------------------------
	
	# ---------- TURN BACK WAIST --------------------------
	backLR = joy.axes[7]	# D-pad up/down

	if (abs(backLR) > 0.1) :
	    if (backLR > 0.1) :
	        self.waist2 = self.waist2 - 5
	    elif (backLR < -0.1) :
	        self.waist2 = self.waist2 + 5
	    self.publishWaist()
	
	# ------------ RESET WAISTS --------------------------
	startBtn = joy.buttons[7]
	if (startBtn == 1):
	    self.waist1 = 0
	    self.waist2 = 0
	    self.publishWaist()

	# -----------------------------------------------------
	# ---------- PENDULUM ARM CONTROL - in pairs ----------
	Ybtn = joy.buttons[3]
	#LT = joy.axes[5]
	LB = joy.buttons[4]
	#RT = joy.axes[4]
	RB = joy.buttons[5]
	
	if (Ybtn == 1):
	    self.pendAction = 0
	    self.publishPend()

	# Choose pendulum arm (1-6)
	if (Ybtn == 1 and self.pendButtonState == 0) :
	    self.pendSelect += 1
	    self.pendButtonState = 1
	    self.pendAction = (self.pendSelect % 6) + 1
	    self.publishPend()
	elif (Ybtn == 0) :
	    self.pendButtonState = 0
	
	# up/down
	if (LB == 1 or RB == 1) :
	    if (LB == 1) :
	        self.pendAction = 8
		self.publishPend()
	    elif (RB == 1) :
	        self.pendAction = 9
		self.publishPend()
	#elif (LB == 0 and RB == 0 and not self.pendAction == 0) :
	#    self.pendAction = 0
	#    self.publishPend()
	#self.publishPend()
	
	# ------ CONTROL ALL PEND ARMS (TOGGLE UP/DOWN) ----
	optBtn = joy.buttons[6]
	if (optBtn == 1 and self.allpendstate == 0):
	    self.allpendstate = 1
	elif (optBtn == 0 and self.allpendstate == 1):
	    self.allpendstate = 0
	
	if (optBtn == 1 and LB == 1 and self.allpendstate == 1 and self.takeover == 0):
	    self.pendAction = 1
	    self.publishPend()
	    time.sleep(0.1)
	    self.pendAction = 8
	    self.publishPend()
	    time.sleep(0.1)
	    self.pendAction = 2
	    self.publishPend()
	    time.sleep(0.1)
	    self.pendAction = 8
	    self.publishPend()
	    time.sleep(0.1)
	    self.pendAction = 3
	    self.publishPend()
	    time.sleep(0.1)
	    self.pendAction = 8
	    self.publishPend()
	    time.sleep(0.1)
	    self.pendAction = 4
	    self.publishPend()
	    time.sleep(0.1)
	    self.pendAction = 8
	    self.publishPend()
	    time.sleep(0.1)
	    self.pendAction = 5
	    self.publishPend()
	    time.sleep(0.1)
	    self.pendAction = 8
	    self.publishPend()
	    time.sleep(0.1)
	    self.pendAction = 6
	    self.publishPend()
	    time.sleep(0.1)
	    self.pendAction = 8
	    self.publishPend()
	    time.sleep(0.1)
	    self.allpendstate = 0
	elif (optBtn == 1 and RB == 1 and self.allpendstate == 1 and self.takeover == 0):
	    self.pendAction = 1
	    self.publishPend()
	    time.sleep(0.1)
	    self.pendAction = 9
	    self.publishPend()
	    time.sleep(0.1)
	    self.pendAction = 2
	    self.publishPend()
	    time.sleep(0.1)
	    self.pendAction = 9
	    self.publishPend()
	    time.sleep(0.1)
	    self.pendAction = 3
	    self.publishPend()
	    time.sleep(0.1)
	    self.pendAction = 9
	    self.publishPend()
	    time.sleep(0.1)
	    self.pendAction = 4
	    self.publishPend()
	    time.sleep(0.1)
	    self.pendAction = 9
	    self.publishPend()
	    time.sleep(0.1)
	    self.pendAction = 5
	    self.publishPend()
	    time.sleep(0.1)
	    self.pendAction = 9
	    self.publishPend()
	    time.sleep(0.1)
	    self.pendAction = 6
	    self.publishPend()
	    time.sleep(0.1)
	    self.pendAction = 9
	    self.publishPend()
	    time.sleep(0.1)
	    self.allpendstate = 0
	
	
	# -----------------------------------------------------

	# ---------- XBOX TAKEOVER --------------------------
	Xbtn = joy.buttons[2]
	if (Xbtn == 1 and self.takeoverButtonState == 0):
	    self.takeoverButtonState = 1
	    if (self.takeover == 0) :
		self.takeover = 1
	    else :
		self.takeover = 0
	    self.publishTakeover()
	else :
	    self.takeoverButtonState = 0
	    
	
	# -----------------------------------------------------
        # Calculate motor action and publish
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
	if not self.action == 4 and not self.oldAction == 4 :        
	    # action_msg is of type Int64
            action_msg = Int64()
            # pack action
            action_msg.data = self.action
            # pub
	    if self.takeover == 1:
                self.pub1.publish(action_msg)
    
    def publishWaist(self):	
	waist_msg = Point()
	waist_msg.x = self.waist1
	waist_msg.y = self.waist2
	if self.takeover == 1:
	    self.pubWaist.publish(waist_msg)

    def publishPend(self):
	pend_msg = Int64()
	pend_msg = self.pendAction
	#if self.takeover == 1:
	self.pubPend.publish(pend_msg)

    def publish_pwm(self):
	pwm_msg = Int64()
	pwm_msg.data = self.pwm
	if self.takeover == 1:
	    self.pub2.publish(pwm_msg)

    def publishTakeover(self):
	takeover_msg = Int64()
	takeover_msg.data = self.takeover
	#if self.takeover == 1:
	self.pubTakeoverGoal.publish(takeover_msg)

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
