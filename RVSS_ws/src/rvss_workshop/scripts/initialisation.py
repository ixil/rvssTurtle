#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rvss_workshop.msg import cylDataArray
from rvss_workshop.msg import startstop
import numpy as np

import sys, select, termios, tty
from threading import Thread, Lock

from time import sleep

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

key = 0

class initialisation():
	def __init__(self):
		global key
		self.target_dist = 0
		
		self.target_ang = 0
		self.state = 1 # test
		
		#self.pose_sub = rospy.Subscriber("/pose", kalmanLocalizationPose, self.callbackPose)
		sub_odom = rospy.Subscriber('/odom', Odometry, self.callbackPose)
		self.image_sub = rospy.Subscriber("/cylinderTopic",cylDataArray,self.callback)
		self.pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
		self.pub_end = rospy.Publisher('/initialise', startstop, queue_size=1)
		self.twist = Twist()
		self.twist.linear.x = 0.0
		self.twist.angular.z = 0.0
		self.pub.publish(self.twist)
		
		# saved Kalman state estimate (robot pose)
		self.x = 0.0
		self.y = 0.0
		self.ang = 0.0
		self.x_old = 0
		self.y_old = 0.0
		
		# Seen closest cylinder (Z, X, label)
		self.cyl_z = 0
		self.cyl_x = 0
		self.cyl_lab = 0
		self.cyl_seen = False
		self.pub_end.publish(False)
		
		# list of found cylinders
		self.found = []
		
		# mutex
		self.mutex = Lock()

	def callbackPose(self, msg):
		#if(self.mutex.acquire(False)):
		if(True):
			# For odom
			self.x = msg.pose.pose.position.x
			self.y = msg.pose.pose.position.y
			self.ang = np.arccos(msg.pose.pose.orientation.w)*2.0
			
			# For Kalman
			'''self.x = msg.stateMean[0]
			self.y = msg.stateMean[1]
			self.ang = msg.stateMean[2]'''
			
			if (self.state==0):
				'''if (len(self.found)==2):
					print "done", self.found
					self.pub_end.publish(True)
				elif (self.cyl_seen) and (not (self.cyl_lab in self.found)):
					# append into found cylinder
					self.found.append(self.cyl_lab)
				
					# save target angle
					self.target_ang = self.ang + np.arctan2(-self.cyl_x, self.cyl_z)
					if self.target_ang > (np.pi):
						self.target_ang -= 2*np.pi
					elif self.target_ang < -(np.pi):
						self.target_ang += 2*np.pi
				
					# save target (x,y) - 20% distance to prevent bumping
					self.target_dist = np.sqrt((self.cyl_z**2) + (self.cyl_x**2))*0.7 - 0.1
					self.x_old = self.x
					self.y_old = self.y
				
					# change state
					self.state = 1
				
					print "Going to: ", self.cyl_lab
			
				else: # turn to find a cylinder
					self.twist.linear.x = 0.0
					self.twist.angular.z = 0.5
					self.pub.publish(self.twist)'''
				print "done", self.found
				self.pub_end.publish(True)
					
			elif (self.state==1): # Turn to face cylinder
				'''ang_diff = self.target_ang - self.ang
				if ang_diff > (np.pi):
					ang_diff -= 2.0*np.pi
				elif ang_diff < -(np.pi):
					ang_diff += 2.0*np.pi
				print "Going to: ", self.cyl_lab, " Angle diff: ", ang_diff
			
				#if (ang_diff<-0.15) or (ang_diff>2.5 or ang_diff<-2.5):
				#	self.twist.linear.x = 0.0
				#	self.twist.angular.z = -0.5
				#	self.pub.publish(self.twist)
				#el
				if (abs(ang_diff)>0.15):
					self.twist.linear.x = 0.0
					self.twist.angular.z = 0.5
					self.pub.publish(self.twist)
				else: 
					self.twist.linear.x = 0.0
					self.twist.angular.z = 0.0
					self.pub.publish(self.twist)
					self.state = 0'''
				count = 0
				self.twist.linear.x = 0.0
				self.twist.angular.z = 0.5
				self.pub.publish(self.twist)
				count += 1
				print "state 1"
				state = 2
					
			elif (self.state==2): # move towards
				'''distance = np.sqrt((self.x - self.x_old)**2 + (self.y - self.y_old)**2)
				print "Going to: ", self.cyl_lab, " Distance: ", self.target_dist - distance
				if (abs(distance - self.target_dist)>0.1):
					self.twist.linear.x = 0.2
					self.twist.angular.z = 0
					self.pub.publish(self.twist)
				else:
					self.twist.linear.x = 0
					self.twist.angular.z = 0
					self.pub.publish(self.twist)
					self.state = 0'''
				count = 0
				
				self.twist.linear.x = 0.0
				self.twist.angular.z = -0.5
				self.pub.publish(self.twist)
				count += 1
				print "state2"
				state = 0
			#self.mutex.release()
	def callback(self, msg):
		if (len(msg.cylinders)>0):
			dist = []
			for i in range(len(msg.cylinders)):
				dist.append(msg.cylinders[i])
			min_ind = np.argmin(dist)
			self.cyl_z = msg.cylinders[min_ind].Zrobot
			self.cyl_x = msg.cylinders[min_ind].Xrobot
			self.cyl_lab = msg.cylinders[min_ind].label
			self.cyl_seen = True
		else:
			self.cyl_seen = False

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('init_node')
    init = initialisation()
    
    try:
        while(1):
            global key
            key = getKey(settings)
            sleep(0.01)
            if (key == '\x03'):
            	break

    except:
        print e

    finally:
        pass

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

