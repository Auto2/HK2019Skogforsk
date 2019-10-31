#!/usr/bin/env python

## keyboard node to control the robot with WSAD
## publishes an action ID that is then taken by motor_controller_node
## printed feedback is a bit weird 
import rospy 					# to be able to use python code in ros
from std_msgs.msg import Int64 			# Datatype in published message (pub to motor_action)
from pynput import keyboard

class KeyboardWsadNode:
    def __init__(self):
        # node init 
        rospy.init_node('keyboard_wsad_node', anonymous=True)

        self.x = 0 # left = -1, right = +1 # bor nog andras?
        self.y = 0 # backward = +1, forward = -1
	self.pwm = 10;
	self.pendulumAction = 0;

        # actions
        self.action = 5 # 4 = idle, changed from 5, see action_recalc below

        # publisher obj
        self.pub1 = rospy.Publisher('motor_action', Int64, queue_size=1) # publish message
	self.pub2 = rospy.Publisher('motor_pwm',Int64, queue_size=1)
	self.pubPendulum = rospy.Publisher('pendulum_action', Int64, queue_size = 1) #should be replaced with a custom message once the power supply and fuses have been upgraded (not in this project)

        # loop rate
        rate = rospy.Rate(10) # checks if a button is pressed (10Hz)

        # kb listener setup
        with keyboard.Listener(on_press=lambda key: self.on_press(key), on_release=lambda key: self.on_release(key)) as listener:
            listener.join() # listens to keyboard inputs

    def on_press(self, key):
	flag = 0
        try:
            print('')

            if key.char == 'q' or key == keyboard.Key.esc:
                exit()
            elif key.char == 'w':
                self.y = 1
		flag = 0
                print('y = {0}'.format(self.y))
            elif key.char == 's':
                self.y = -1
		flag = 0
                print('y = {0}'.format(self.y))
            elif key.char == 'a':
                self.x = -1
		flag = 0
                print('x = {0}'.format(self.x))
            elif key.char == 'd':
                self.x = 1
		flag = 0
                print('x = {0}'.format(self.x))

            elif key.char <= '9' and key.char >= '1': #if 1 <= keyboard input <= 9
		self.pendulumAction = ord(key.char)-48 #converts input to int in base 10              
                self.publish_pendulum()
		flag = 1



	    elif key.char == '+':
		if self.pwm < 90:
		    self.pwm = self.pwm + 10
		    print('pwm = {0}'.format(self.pwm))
		    self.publish_pwm()
		    flag = 1
	    elif key.char == '-':
		if self.pwm > 10:
		    self.pwm = self.pwm - 10
		    print('pwm = {0}'.format(self.pwm))
		    self.publish_pwm()
		    flag = 1


            
	    if flag == 0:
		self.action_recalc()  	# function below 
                self.publish() 		# same

        except AttributeError: 		# if not wsad+-q (wont happen)
            pass

    # on button release set vaiables x and y to 0
    def on_release(self,key):  
	flag = 0
        try:
            print('')

            if key.char == 'w' or key.char == 's':
                self.y = 0
            elif key.char == 'a' or key.char == 'd':
                self.x = 0

            elif key.char <= '9' and key.char >= '1':
                self.pendulumAction = 0;
                self.publish_pendulum()
		flag = 2
                print "pendulum action = 0"


	    elif key.char == '+' or key.char == '-':
                print('pwm = {0}'.format(self.pwm))
		self.publish_pwm()
		flag = 1

	    if flag == 0:
		self.action_recalc()
		self.publish()
        except AttributeError:
            pass

    def action_recalc(self):
        # actions - imagine a keypad:
        # 6 7 8     FL F FR
        # 3 4 5  =  L  I  R
        # 0 1 2     BL B BR
        # F = forward, B = backward, L = left, R = right, I = idle
        self.action = self.x + 3*self.y + 4 # see numbers on "keypad" and directions above
	if self.action == 3: 
            self.action = 4      
	elif self.action == 5:	
	    self.action = 4      

        print('action: {0}'.format(self.action))

    def publish(self):
        # action_msg is of type Int64
        action_msg = Int64()
        # pack action
        action_msg.data = self.action
        # pub
        self.pub1.publish(action_msg)


	
    
    def publish_pwm(self):
	pwm_msg = Int64()
	pwm_msg.data = self.pwm
	self.pub2.publish(pwm_msg)

    def publish_pendulum(self):
        #action_msg is of type Int64
        pendulum_msg = Int64()
        #pack action
        pendulum_msg.data = self.pendulumAction
        # pub
        self.pubPendulum.publish(pendulum_msg)
  

# If no key is pressed -> nothing/pass/nemas
if __name__ == '__main__':
    try:
        kn = KeyboardWsadNode() 
    except rospy.ROSInterruptException:
        pass


# If no key is pressed -> nothing/pass/nemas
if __name__ == '__main__':
    try:
        kn = KeyboardWsadNode() 
    except rospy.ROSInterruptException:
        pass
