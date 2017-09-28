#!/usr/bin/env python
import rospy
import time
import random
import math
from numpy import *
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class turtleSwarm():
	def __init__(self):
		rospy.init_node('turtleswarm', anonymous=True)

class turtleLead():

	def __init__(self):
		rospy.init_node('turtleLead', anonymous=True)
		self.vel_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=5)
		self.pos_subscriber = rospy.Subscriber('/turtle1/pose',Pose,self.pose_callback)
		self.rate = rospy.Rate(5)
		self.waypoints = array([[3,3],[5,10],[10,5]])
		self.curWaypt = 0
		(self.numWaypt,null) =  shape(self.waypoints)
		self.numWaypt = self.numWaypt -1;
		print self.numWaypt


	def pose_callback(self,data):
		self.pose = data
		print data
		


	def sendVel(self,vx,omega):
		vel_msg = Twist()
		vel_msg.linear.x = vx
		vel_msg.angular.z = omega
		self.vel_publisher.publish(vel_msg)

	def moveTo_wayPt (self):
		wayPt = self.waypoints[self.curWaypt]
		v = self.calcVel(wayPt);
		self.sendVel(v[0],v[1])

	def calcVel(self,wayPt):
		#wayPt in form [x,y]
		pose = self.pose
		epsilon =0.3
		xTurt = pose.x; yTurt = pose.y; thetaTurt = pose.theta; 
		xWp = wayPt[0]; yWp = wayPt[1];
		xVect = xWp - xTurt
		yVect = yWp - yTurt

		mag = sqrt(xVect**2 + yVect**2)
		vGlob = array([[xVect],[yVect]])/mag

		R = array([[cos(thetaTurt),sin(thetaTurt)],[-sin(thetaTurt),cos(thetaTurt)]])
		T = dot(array([[1,0],[0,1/epsilon]]),R)

		vOut = dot(T,vGlob)
		if ((abs(xVect)  < 0.2) and (abs(yVect) < 0.2)):
			vOut = array([[0],[0]])
			if self.curWaypt < self.numWaypt:
				self.curWaypt = self.curWaypt + 1
		return vOut

class turtleFol():

if __name__ =='__main__':
	lead = turtleLead()
	vel_msg = Twist()
	vel_msg.linear.x = 1
	timenow = time.time()
	startT = time.time()
	lead.rate.sleep()
	while(time.time()-startT < 20):
		lead.moveTo_wayPt()
		lead.rate.sleep()
		pass
	
