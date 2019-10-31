#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Int64 #used to import desired turn radius value
from geometry_msgs.msg import Point #x, y used for front and rear waist angle (float 64) respectibely

##-----------------------------DESCRIPTION---------------------------------##
#Calculates the angle for each waist for a given desired turn radius and
#current configuration of the test rig's pendulum arms (the geometry which 
#define the rotational axes of the dirving motors). 
#
#Subscribes to "cmd_turn_radius": this sets the desired turn radius in mm 
#(positive for turning right, negative for turning left)
#
#Subscribes to "motor_action": this is user input which tells if the testrig 
#should turn more or less in a specified direcion (increment turn radius)
#
#Subscribes to "waist_angles": this is a point message which corresponds to
#the measured angles of the waists.
#[x, y, z] = [front_angle, rear_angle, NULL]
#
#
#Publishes "desired_radius": This is what the node thinks is the current des-
#ired turn radius in mm.
#
#Publishes "cmd_wasit_twists": This is a Point message 
#positive angles => turn right
#negative angles => turn left
#[x, y, z] = [front_waist_angle, rear_waist_angle, desired_radius]
#
#Publishes "actual_radius": which is the calculated turn radius derived
#from the measured waist angles
##-----------------------------DESCRIPTION---------------------------------##


##-----------------------------INIT---------------------------------##
rospy.init_node('desired_turn_radius_to_waist_angles')
pubDesRad = rospy.Publisher('desired_radius', Int64, queue_size = 2)
pubAngles = rospy.Publisher('cmd_waist_twists', Point, queue_size = 1)
pubRad = rospy.Publisher('actual_radius', Int64, queue_size = 1)
rate = rospy.Rate(10)
##-----------------------------INIT---------------------------------##


R_des = 1000 #desired turn radius from center of body along middle wheel axle (mm)

##----------------CONFIGURATION PARAMETERS----------------##
#all measurements in mm and seen from top plane   
L = 500 #distance between the two waists
d_f = 235 #distance between front waist and front wheel axle
d_m = 285 #distance between front waist and middle wheel axle
d_r = 235 #distance between front waist and rear wheel axle
allowedTwist = [0, 45] #maximum twist of waists
tolerance = 0.01 #percentage based error tolerance for acceptable turn radius
iterationLimit = 1507 #maximum amount of iterations
MAX_TURN_RADIUS = 4000 #mm
MIN_TURN_RADIUS = 500 #mm
turnRate = 10 #mm/increment
##--------------------------------------------------------##



#calculate the turn radius based on waist twist and distance between the waist and motor axes in the top plane
def calculateTurnRadius(d_e, d_m, angleDegrees):
    angle = angleDegrees * np.pi / 180 #angle of waist in rad
    oppositeAngle = np.pi/2 - angle

    R = np.tan(oppositeAngle) *(d_m + d_e * (np.cos(angle) + np.sin(angle)/np.tan(oppositeAngle)))

    return R

    
def iterateWaistTwist(): #input desired turn radius
    global R_des, d_f, d_m #uses the global desired turn radius in case it gets updated

    iterations = 0 #counter for the number of iterations performed
    
    foundAlpha = False
    foundBeta = False
    
    permissibleAlpha = True
    permissibleBeta = True

    
    alpha = allowedTwist[1]/2 #initial angle guess
    beta = allowedTwist[1]/2 #initial angle guess
    
    while iterations < iterationLimit:
        Rf = calculateTurnRadius(d_f, d_m, alpha)
        Rr = calculateTurnRadius(d_f, L - d_m, beta) #change "d_f" to "d_r" in case front and rear config isn't the same
        
        #print("Calculated turn radius: " + str(Rf) + " Using front angle: " + str(alpha))
        #print("Calculated turn radius: " + str(Rr) + " Using rear angle: " + str(beta))
        
        dRf = Rf - abs(R_des)
        if  abs(dRf) >= tolerance:
            if dRf > 0 and alpha >= allowedTwist[0] and alpha <= allowedTwist[1]:
                alpha += 0.1
            elif dRf < 0:
                alpha -= 0.1
            else:
                #print("Non-permissible front angle requirement to achieve desired turn radius")
                permissibleAlpha = False
        else:
            pass
            #print("Found Acceptable alpha")
            
        dRr = Rr - abs(R_des)
        if  abs(dRr) >= tolerance:
            if dRr > 0 and beta >= allowedTwist[0] and beta <= allowedTwist[1]:
                beta += 0.1
            elif dRr < 0:
                beta -= 0.1
            else:
                #print("Non-permissible rear angle requirement to achieve desired turn radius")
                permissibleBeta = False
        else:
            pass
            #print("Found Acceptable beta")
            
        if foundAlpha and foundBeta: #no point continuing if both are found
            break 
        
        iterations += 1

    if not permissibleAlpha:
        pass
        #print("Non-permissible front angle requirement to achieve desired turn radius")
    if not permissibleBeta:
        pass
        #print("Non-permissible rear angle requirement to achieve desired turn radius")
    if R_des < 0:
	alpha = (-1) * alpha
	beta = (-1) * beta    
    return alpha, beta

def desiredRadiusCallback(request): #gets Int64 as input
    global R_des	
    R_des = request.data
    pubDesRad.publish(R_des)

def incrementDesiredRadiusCallback(action):
    global R_des
    global turnRate
    
    if action.data == 8 or action.data == 2: #turn right
	if R_des == 0:
            R_des = MAX_TURN_RADIUS
        elif R_des == MIN_TURN_RADIUS:
            R_des = MIN_TURN_RADIUS
        elif R_des == (-1) * MAX_TURN_RADIUS:
            R_des = 0
	elif R_des - turnRate > (-1) * MIN_TURN_RADIUS and R_des - turnRate <= 0:
            R_des = 0
        else:
            R_des = R_des - turnRate #decrease radius to increase turning angle
            
    if action.data == 6 or action.data == 0: #turn left
        if R_des == 0:
            R_des = (-1) * MAX_TURN_RADIUS
        elif R_des == (-1) * MIN_TURN_RADIUS:
            R_des = (-1) * MIN_TURN_RADIUS
        elif R_des == MAX_TURN_RADIUS:
            R_des = 0
	elif R_des - turnRate < MIN_TURN_RADIUS and R_des - turnRate >= 0:
            R_des = 0
        else:         
            R_des = R_des + turnRate #increase radius to increase turning angle (remember that the turn radius is negative when turning left)
    pubDesRad.publish(R_des)

def angleFeedbackCallback(angles):
    a = angles.x
    b = angles.y

    Ra = calculateTurnRadius(d_f, d_m, a)
    Rb = calculateTurnRadius(d_f, L - d_m, b) #change "d_f" to "d_r" in case front and rear config isn't the same
    R_real = (Ra + Rb) / 2

    pubRad.publish(R_real) #publish the calculated turn radius from the measured waist angles
        


def doStuff():
	frontAngle, rearAngle = iterateWaistTwist() #positive angles mean that the testrig should turn clockwise and negative counter-clockwise
	#print("Front angle: " + str(frontAngle) + " Rear Angle: " + str(rearAngle))
	angles_msg = Point()
	angles_msg.x = frontAngle
	angles_msg.y = rearAngle
	angles_msg.z = R_des
	pubAngles.publish(angles_msg)
	

def main():

	subRadius = rospy.Subscriber('cmd_turn_radius', Int64, desiredRadiusCallback)
	subDeltaRadius = rospy.Subscriber('motor_action', Int64, incrementDesiredRadiusCallback)
        subActualWaistAngles = rospy.Subscriber('waist_angles', Point, angleFeedbackCallback)
	while not rospy.is_shutdown():
		doStuff()
		rate.sleep()

if __name__ == '__main__':
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass
