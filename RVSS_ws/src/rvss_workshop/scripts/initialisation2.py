#!/usr/bin/env python2

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from rvss_workshop.msg import kalmanLocalizationPose
from rvss_workshop.msg import reachedNextPose
from rvss_workshop.msg import nextPose
from nav_msgs.msg import Odometry
from rvss_workshop.msg import startstop
import time

from threading import Thread, Lock

class Execution:

  #initialise the publisher and subscriber
  def __init__(self):
    self.state_sub = rospy.Subscriber("/odom", Odometry, self.callbackPose) 
    self.pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size=1)
    self.pub_end = rospy.Publisher('/initialise', startstop, queue_size=1)
    self.state = 'waiting'
    self.done = 0
    self.receivedNextPose = 1 # testing
    self.nextPose = np.array([[1.0],[1.0],[0.0]])
    self.pub_end.publish(False)
    
    self.mutex = Lock()
  
  def computeCommands(self):
   	#compute angle & distance to poseFinal
		dX = self.nextPose[0]-self.startPose[0]
		dY = self.nextPose[1]-self.startPose[1]
		distance = np.sqrt(dX**2 + dY**2)
		angle    = np.arctan2(dY,dX) - self.startPose[2]
		if angle < 0:
			self.turnSign = -1
		else:
			self.turnSign = 1

		#compute duration of rotation and translation
		self.angularVelocity = 0.2
		self.linearVelocity  = 0.2
		self.rotationDuration    = np.abs(angle/self.angularVelocity) 
		self.translationDuration = np.abs(distance/self.linearVelocity)	
		#get these values from calibration of dead reckoning
		self.rotationDuration*=1
		self.translationDuration*=1
		
		#publish commands based on state
		self.done = 0 
  
    
  # callback for robotState
  def callbackPose(self, data):
  	if self.state=='waiting':
  		#self.startPose = data.stateMean
  		self.startPose = np.array([[0.0],[0.0],[0.0]]) #JUST FOR TESTING
  		self.receivedStartPose = 1
  		if self.receivedNextPose:
  			self.state = 'rotating'
  			self.tStartRotation = rospy.get_time()
  			self.computeCommands()
      #self.mutex.release()#is this necessary?
               
if __name__ == '__main__':
    rospy.init_node('init', anonymous=True)
    execution = Execution()
    print('Executing motion...')
    
    try:
      while (not execution.done):		
				if execution.state=='rotating':
					while rospy.get_time() < execution.tStartRotation + execution.rotationDuration:
						twist = Twist()
						twist.linear.x = 0
						twist.angular.z = execution.turnSign*execution.angularVelocity
						execution.pub.publish(twist)
						print('rotating')
					execution.state = 'translating'
					execution.tStartTranslation = rospy.get_time()
				if execution.state=='translating':
					while rospy.get_time() < execution.tStartTranslation + execution.translationDuration:
						twist = Twist()
						twist.linear.x = execution.linearVelocity
						twist.angular.z = 0
						execution.pub.publish(twist)
						print("rotating")  
				 	#execution.state = 'waiting'
				 	execution.done = 1 #JUST FOR TESTING
      execution.pub_end.publish(True)
    except KeyboardInterrupt:
        print("Shutting down")
