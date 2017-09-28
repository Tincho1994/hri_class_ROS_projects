#!/usr/bin/env python
import rospy
import time
import random
import math
from numpy import *
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class turtleLead():

	def __init__(self):
		rospy.init_node('drunkturtle_enebriator', anonymous=True)
		self.vel_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=5)
		self.rate = rospy.Rate(5)

	def sendVel(self,vx,omega):
		vel_msg = Twist()
		vel_msg.linear.x = vx
		vel_msg.angular.z = omega
		self.vel_publisher.publish(vel_msg)

	def calcVel(self,pose,wayPt):
		#wayPt in form [x,y]
		epsilon = 0.1;
		xTurt = pose[0]; yTurt = pose[1]; thetaTurt = radians(pose[2]); 
		xWp = wayPt[0]; yWp = wayPt[1];
		xVect = xWp - xTurt
		yVect = yWp - yTurt
		mag = sqrt(xVect**2 + yVect**2)
		vGlob = array([[xVect],[yVect]])/mag

		R = array([[cos(thetaTurt),sin(thetaTurt)],[-sin(thetaTurt),cos(thetaTurt)]])
		T = dot(array([[1,0],[0,1/epsilon]]),R)

		vOut = dot(T,vGlob)


if __name__ =='__main__':
	drunkAF = turtleLead()
	vel_msg = Twist()
	vel_msg.linear.x = 1
	timenow = time.time()

	pose = array([0,0,0])
	wayPt = array([1.,1.])
	drunkAF.calcVel(pose,wayPt)
