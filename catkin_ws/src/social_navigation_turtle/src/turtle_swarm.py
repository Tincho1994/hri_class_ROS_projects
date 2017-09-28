#!/usr/bin/env python
import rospy
import time
import random
import numpy as np
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

	def calcVel(self,pose,wayPt)
		#wayPt in form [x,y]
		xTurt = pose[1]; yTurt = pose[2]; thetaTurt = pose[3]; 
		xWp = pose[1]; yWp = wayPt[2];
		xVect = xWp - xTurt
		yVect = yWp - yTurt
		vGlob = np.array([xVect],[yVect])/math.sqrt(xVect^2 + yVect^2)
		print vGlob



if __name__ =='__main__':
	drunkAF = drunkturtle()
	vel_msg = Twist()
	vel_msg.linear.x = 1
	timenow = time.time()
	while(time.time()-timenow < 5):
		drunkAF.stumbleForward(0.5)
		drunkAF.rate.sleep()
	pose = np.array([0,0,0])
	wayPt = np.array([1,1])
	drunkAF.calcVel(pose,wayPt)
