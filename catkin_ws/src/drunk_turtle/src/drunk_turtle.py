#!/usr/bin/env python
import rospy
import time
import random
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class drunkturtle():

	def __init__(self):
		rospy.init_node('drunkturtle_enebriator', anonymous=True)
		self.vel_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=5)
		self.rate = rospy.Rate(5)

	def sendVel(self,vx,omega):
		vel_msg = Twist()
		vel_msg.linear.x = vx
		vel_msg.angular.z = omega
		self.vel_publisher.publish(vel_msg)

	def stumbleForward(self,vx):
		omega = random.randint(-10,10)
		omega = float(omega)/5
		self.sendVel(vx,omega)


if __name__ =='__main__':
	drunkAF = drunkturtle()
	vel_msg = Twist()
	vel_msg.linear.x = 1
	timenow = time.time()
	while(time.time()-timenow < 15):
		drunkAF.stumbleForward(0.5)
		drunkAF.rate.sleep()
