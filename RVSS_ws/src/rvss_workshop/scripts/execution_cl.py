#!/usr/bin/env python2

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from rvss_workshop.msg import kalmanLocalizationPose
from rvss_workshop.msg import reachedNextPose
from rvss_workshop.msg import nextPose
import time

from threading import Thread, Lock


class Execution:

  #initialise the publisher and subscriber
  def __init__(self):
    self.state_sub = rospy.Subscriber("/pose", kalmanLocalizationPose, self.callbackPose)
    self.nextPose_sub = rospy.Subscriber("/nextPose", nextPose, self.callbackNextPose)
    self.pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size=1)
    self.reachedNextPose_pub = rospy.Publisher("/reachedNextPose",reachedNextPose,queue_size=1)
    self.state = 0
    self.Tdistance = np.inf
    self.Tangle = np.inf
    self.nextPose = nextPose
    self.rx = 0
    self.ry = 0
    self.rt = 0
    self.rx_old = 0
    self.ry_old = 0
    self.rt_old = 0
    
    self.t_prev = rospy.get_time() #time.clock()
    self.t_dur = 0
    self.mutex = Lock()
    
 # callback for robotState
  def callbackPose(self, data):
      if(self.mutex.acquire(False)):
        self.lastMessage = data
        print self.lastMessage.stateCovariance
        if(np.linalg.det(np.reshape(self.lastMessage.stateCovariance,[3,3]))<1):
          # robot absoulte pose
          self.rx = self.lastMessage.stateMean[0]
          self.ry = self.lastMessage.stateMean[1]
          self.rt = self.lastMessage.stateMean[2]

        if(self.state==1):
          print "going"
          ang_diff = self.Tangle - self.rt
          if ang_diff > (np.pi):
            ang_diff -= 2.0*np.pi
          elif ang_diff < -(np.pi):
            ang_diff += 2.0*np.pi
        
          if (abs(ang_diff) > 0.1):
            twist = Twist()
            # rotation
            twist.linear.x = 0
            twist.angular.z = 0.3
            self.pub.publish(twist)
          else:
            self.state = 2
            self.t_prev = rospy.get_time() #time.clock()
          
        if(self.state==2):
          #distance = np.sqrt((self.rx - self.rx_old)**2 + (self.ry - self.ry_old)**2)
          #print "target: ", (self.Tdistance - distance)
          #if ((self.Tdistance - distance) > 0.1):
          t = rospy.get_time() - self.t_prev  #time.clock() - self.t_prev
          print "target t: ", self.t_dur, "current t:", t
          if (t<self.t_dur):
            twist = Twist()
            # linear motion
            twist.linear.x = 0.3
            twist.angular.z = 0
            self.pub.publish(twist)
          else:
            stop = Twist()
            # linear motion
            stop.linear.x = 0.3
            stop.angular.z = 0
            self.pub.publish(stop)
            self.state = 0
            status = reachedNextPose()
            status.reachedNextPose = 1
            self.reachedNextPose_pub.publish(status)
        self.mutex.release()

  # callback for the pose messages
  def callbackNextPose(self,data):
    self.nextPose = data.nextPose
  
    # target distance & angle to goal pose
    self.Tdistance = np.sqrt((self.nextPose[0]-self.rx)**2 + (self.nextPose[1]-self.ry)**2)
    self.t_dur = self.Tdistance / 0.3

    self.Tangle = self.nextPose[2]
    if self.Tangle > (np.pi):
      self.Tangle -= 2.0*np.pi
    elif self.Tangle < -(np.pi):
      self.Tangle += 2.0*np.pi

    # save starting pose
    self.rx_old = self.rx
    self.ry_old = self.ry
    self.rt_old = self.rt
    self.state = 1
    print('publishing commands')
    
               
if __name__ == '__main__':
    rospy.init_node('execute', anonymous=True)
    execution = Execution()
    print('Executing motion...')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
