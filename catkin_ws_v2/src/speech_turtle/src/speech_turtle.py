#!/usr/bin/env python
import rospy
import time
import random
import subprocess
import turtlesim.srv
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from hlpr_speech_msgs.msg import StampedString
class turtleMaster():
	def __init__(self):
		self.numControl = 2
		self.activeTurtle = None
		self.com_subscriber = rospy.Subscriber('/hlpr_speech_commands',StampedString,self.comm_callback)
		rospy.init_node('speechTurtle_master', anonymous=True)
		self.rate = rospy.Rate(5)
		self.commandMap = {"OKAY FIRST TURTLE":"TURTLE_1",
							"SECOND TURTLE":"TURTLE_2",
							"STOP RIGHT THERE":"STOP",
							"GO FORWARD":"FORWARD",
							"REVERSE":"BACK",
							"SPIN LEFT":"TURN_LEFT",
							"TURN TO THE RIGHT":"TURN_RIGHT"}
		self.turtles = {"TURTLE_1":turtle(1,self),"TURTLE_2":turtle(2,self)}

	def run(self):
		self.turtles['TURTLE_1'].executeCommand()
		self.turtles['TURTLE_2'].executeCommand()
		self.rate.sleep()

	def comm_callback(self,msg):
		content = msg.keyphrase
		command = self.commandMap[content]
		print content
		if (command == "TURTLE_1" or command == "TURTLE_2"):
			self.activeTurtle = self.turtles[command]
		else:
			self.activeTurtle.updateCommand(command)



class turtle():

	def __init__(self,turtleNum,master):
		self.master = master
		self.turtleNum = turtleNum
		self.vel_publisher = rospy.Publisher('/turtle'+str(self.turtleNum)+'/cmd_vel',Twist, queue_size=5)
		self.lastCommand = None
		self.rate = rospy.Rate(5)

	def updateCommand(self,command):
		self.lastCommand = command

	def executeCommand(self):
		if self.lastCommand == 'STOP':
			self.sendVel(0,0)
		elif self.lastCommand == 'FORWARD':
			self.sendVel(0.35,0)
		elif self.lastCommand == 'BACK':
			self.sendVel(-0.35,0)
		elif self.lastCommand == 'TURN_LEFT':
			self.sendVel(0,0.3)
		elif self.lastCommand == 'TURN_RIGHT':
			self.sendVel(0,-0.3)

	def sendVel(self,vx,omega):
		vel_msg = Twist()
		vel_msg.linear.x = vx
		vel_msg.angular.z = omega
		self.vel_publisher.publish(vel_msg)


if __name__ =='__main__':
	rospy.wait_for_service('spawn')
	turtleMake = rospy.ServiceProxy('spawn',turtlesim.srv.Spawn)
	turtleMake(10,10,0,'turtle2')
	master = turtleMaster()
	timenow = time.time()
	while(time.time()-timenow < 90):
		master.run()
