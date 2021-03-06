#!/usr/bin/env python
import rospy
import time
import random
import math
import turtlesim
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
		self.waypoints = array([[2,2],[2,10],[5,10],[10,8],[10,2],[5,5]])
		self.curWaypt = 0
		(self.numWaypt,null) =  shape(self.waypoints)
		self.numWaypt = self.numWaypt -1;

	def pose_callback(self,data):
		self.pose = data

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
	def __init__(self,name,otherFol):
		rospy.init_node('turtleLead', anonymous=True)
		self.vel_publisher = rospy.Publisher('/'+name+'/cmd_vel',Twist, queue_size=5)
		self.pos_subscriber = rospy.Subscriber('/'+name+'/pose',Pose,self.pose_callback)
		self.lead_velSub = rospy.Subscriber('/turtle1/cmd_vel',Twist,self.velLead_callback)
		self.lead_posSub = rospy.Subscriber('/turtle1/pose',Pose,self.posLead_callback)
		self.otherFol_sub = {}
		self.otherPos = {}
		for oth in otherFol:
			self.otherFol_sub[oth] = rospy.Subscriber('/'+oth+'/pose',Pose,self.posOther_callback)
			self.otherPos[oth] = Pose()
		self.rate = rospy.Rate(5)
		self.leadVel = Twist()
		self.leadPos = Pose()

	def pose_callback(self,data):
		self.pose = data
	def velLead_callback(self,data):
		thetaTurt = self.leadPos.theta
		data.linear.x = cos(thetaTurt)*data.linear.x
		data.linear.y = sin(thetaTurt)*data.linear.x
		self.leadVel  = data
	def posLead_callback(self,data):
		self.leadPos = data
	def posOther_callback(self,data):
		# Get message info and topic
		info = data._connection_header
		topic = info['topic']
		# Get name of turtle to which message belongs to
		turtle = topic.split("/")[1]
		self.otherPos[turtle] =  data

	def sendVel(self,vx,omega):
		vel_msg = Twist()
		vel_msg.linear.x = vx
		vel_msg.angular.z = omega
		self.vel_publisher.publish(vel_msg)

	def moveToLead(self):
		pose = self.pose
		epsilon =1
		xTurt = pose.x; yTurt = pose.y; thetaTurt = pose.theta; 
		xWp = self.leadPos.x; yWp = self.leadPos.y;
		xVect = xWp - xTurt
		yVect = yWp - yTurt

		mag = sqrt(xVect**2 + yVect**2)
		if mag < 1:
			vGlob = array([[0],[0]])
		elif mag < 4:
			vGlob = array([[xVect],[yVect]])/(mag*1.25)
		else:
			vGlob = array([[xVect],[yVect]])/(mag*0.8)

		vGlob = self.avoidCollision(vGlob)
		cmds = self.convertVel(vGlob)
		self.sendVel(cmds[0],cmds[1])
	def avoidCollision(self,vel):
		x = self.pose.x
		y = self.pose.y
		for oth in self.otherPos:
			x2 =  self.otherPos[oth].x
			y2 = self.otherPos[oth].y
			xvect = x-x2
			yvect = y-y2
			dist =  sqrt(xvect**2 + yvect**2)
			if dist < 1:
				vel[0]=vel[0]+xvect/dist
				vel[1]=vel[1]+yvect/dist
		mag = sqrt(vel[0]**2 + vel[1]**2)
		if (mag>0):
			vel = vel/mag
		return vel
	def convertVel(self,vel):
		epsilon =1
		thetaTurt = self.pose.theta

		R = array([[cos(thetaTurt),sin(thetaTurt)],[-sin(thetaTurt),cos(thetaTurt)]])
		T = dot(array([[1,0],[0,1/epsilon]]),R)
		vOut = dot(T,vel)
		
		return vOut

	def copyT(self):
		vOut = self.convertVel()
		self.sendVel(vOut[0],vOut[1])
		


if __name__ =='__main__':
	lead = turtleLead()
	numFollowers = 16
	fol = []*numFollowers
	for i in range(2,numFollowers+2):
		others = []*numFollowers
		others.append('turtle1')
		for j in range(2,numFollowers+2):
			if (j != i):
				others.append('turtle'+str(j))
		fol.append(turtleFol('turtle'+str(i),others))
	
	timenow = time.time()
	startT = time.time()
	lead.rate.sleep()
	while(time.time()-startT < 40):
		lead.moveTo_wayPt()
		for f in fol:
			f.moveToLead()
		lead.rate.sleep()
		pass
	
