#!/usr/bin/env python2

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from rvss_workshop.msg import kalmanLocalizationPose
from rvss_workshop.msg import reachedNextPose
from rvss_workshop.msg import nextPose
from nav_msgs.msg import Odometry
import time

from threading import Thread, Lock

class Execution:

  #initialise the publisher and subscriber
  def __init__(self):
    #self.state_sub = rospy.Subscriber("/odom", Odometry, self.callbackPose) # testing
    self.state_sub = rospy.Subscriber("/pose", kalmanLocalizationPose, self.callbackPose)
    self.nextPose_sub = rospy.Subscriber("/nextPose", nextPose, self.callbackNextPose)
    self.pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size=1)
    self.reachedNextPose_pub = rospy.Publisher("/reachedNextPose",reachedNextPose,queue_size=1)
    self.state = 'waiting'
    self.done = 1
    self.receivedNextPose = 0 # testing
    # self.prevNextPose= np.array([[1.0],[1.0],[-np.pi/4]])
    self.mutex = Lock()
  
  def computeCommands(self):
    #compute angle & distance to poseFinal
    # self.nextPosePrev= self.nextPose
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
  		self.startPose = data.stateMean
  		#self.startPose = np.array([[0.0],[0.0],[0.0]]) #JUST FOR TESTING
  		self.receivedStartPose = 1
  		# if self.receivedNextPose:
  		# 	self.state = 'rotating'
  		# 	self.tStartRotation = rospy.get_time()
  		# 	self.computeCommands()
      #self.mutex.release()#is this necessary?
      
  # callback for the pose messages
  def callbackNextPose(self,data):
    print("NEXT POSE CALLBACK: ",self.state)
    if self.state=='waiting':
      self.nextPose = data.nextPose
      # self.nextPose = self.prevNextPose
      #self.nextPose = np.array([[3.0],[3.0],[0.0]]) #JUST FOR TESTING
      self.receivedNextPose = 1
      if self.receivedStartPose:
  			self.state = 'rotating'
  			self.tStartRotation = rospy.get_time()
  			self.computeCommands()
               
if __name__ == '__main__':
    rospy.init_node('execute', anonymous=True)
    execution = Execution()
    print('Executing motion...')
    
    try:
      while True:
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
  						print("translating")  
            execution.state = 'waiting'
            msg = reachedNextPose()
            msg.reachedNextPose = 1
            execution.reachedNextPose_pub.publish(msg)
            execution.done = 1 #JUST FOR TESTING

    except KeyboardInterrupt:
        print("Shutting down")
