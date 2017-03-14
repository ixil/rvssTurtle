#!/usr/bin/env python2

import os
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from rvss_workshop.msg import kalmanLocalizationPose
from rvss_workshop.msg import cylDataArray
from rvss_workshop.msg import cylMsg
from rvss_workshop.msg import nextPose
from rvss_workshop.msg import startstop
from rvss_workshop.msg import reachedNextPose
from time import sleep
import sys, select, termios, tty

#Functions
from utils import Relative2AbsolutePose
from utils import Relative2AbsoluteXY 
from utils import Absolute2RelativeXY
from utils import pi2pi
from planning import (generateLoop, generateTaskPath)
from utils import findClosestPointIndex

import matplotlib.pyplot as plt

class Main(object):
    #initialise the publisher and subscriber
    def __init__(self):
        self.pose_sub = rospy.Subscriber("/pose", kalmanLocalizationPose, self.callbackPose)
        #self.cyl_sub = rospy.Subscriber("/cylinderTopic",cylDataArray,self.callbackCyl)
        self.obj_sub = rospy.Subscriber("/objTopic",startstop,self.callbackObj)
        self.initialised_sub = rospy.Subscriber("/initialise",startstop,self.callbackInitialisation)
        self.reachedNextPose_sub = rospy.Subscriber("/reachedNextPose",reachedNextPose,self.callbackReachedNextPose)
        self.nextPose_pub = rospy.Publisher("/nextPose", nextPose, queue_size=1)
        self.pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist, queue_size=1)
        self.state = 'initialise'
        home = os.getcwd()
        f = open(home[0:-5]+'/RVSS_ws/src/rvss_workshop/scripts/GT.txt','r')
        self.map = np.loadtxt(f) # 9x2 array, each row contains x and y coordinates of a cylinder
        self.objectPoints = np.empty((0,2)) #initially no objects
        self.objectLabels = []
     #    #path planning settings
     	self.publishNextPose = 1
        self.initialised = False
        self.nBisections = 1
        self.minDistance = 0.1
        self.direction   = 'anticlockwise'
        self.getNewPath = 'True'

    # Callbacks
    def callbackPose(self, msg):
    	print('POSE')
        # print(msg.stateMean)
        # print(msg.stateCovariance)
        self.pose = np.reshape(msg.stateMean,[3,1])
        # self.poseCov = np.reshape(msg.stateCovariance,[9,1])
        self.run()
        
    def callbackReachedNextPose(self, msg):
    	print("Reached next pose AAA")
    	self.publishNextPose = msg.reachedNextPose
    	if self.publishNextPose:
    		# self.iNextPose = (self.iNextPose+1)%self.path.shape[1]
    		self.run()
    		print('Reached next pose')

    def callbackInitialisation(self, msg):
		if msg.startstop==True:
		    self.initialised = True #set in function
		    print('initialised')

    def callbackObj(self, msg):
        for obj in msg.cylinders:
            Zrobot = obj.Zrobot
            Xrobot = obj.Xrobot
            label  = obj.label
            if ((self.state=='explore') & (obj.label not in self.objectLabels) & (not self.getNewPath)):
                print('New Object detected')
                print(obj.label)
	            #convert relative coords to absolute coords
                xyAbs,H1,H2 = Relative2AbsoluteXY(self.pose,[Zrobot,-Xrobot])
                self.objectPoints = np.vstack((self.objectPoints,np.reshape(xyAbs,[1,2])))
                self.objectLabels.append(label)
                print(self.objectPoints)
	            #add xyAbs to self.objectPoints
	            #add obj.label to self.objectLabels
                self.state = 'task'
                self.getNewPath = 'True'
                self.run()

    def run(self):
        # State Machine
        print(self.state)
        if (self.state=='initialise'):
            #perform initialisation routine
            print('state: initialise')
            if self.initialised:
                self.state = 'explore'
                self.getNewPath = True
        elif (self.state=='explore'):
            print('state: explore ',self.publishNextPose)
            if self.getNewPath:
        		#get new path
	            orderedPoses,_ = generateLoop(self.nBisections,self.minDistance,self.direction,
	            								 self.map[range(0,3),:],self.map[range(3,8),:],self.objectPoints)
	            self.iNextPose = findClosestPointIndex(orderedPoses,self.pose)
	            self.path = np.transpose(orderedPoses)
	            print('Generated exploration path')
	            self.getNewPath = False
	            self.publishNextPose = 1
            elif (self.publishNextPose):
	        	self.iNextPose = (self.iNextPose+1)%self.path.shape[1]

	            #display path
	            # ptsIn  = self.map[range(0,3),:]
	            # ptsOut = self.map[range(3,8),:]
	            # ptsObs = self.objectPoints
	            # # plt.figure()
	            # plt.plot(ptsIn[:,0], ptsIn[:,1], 'o',color='blue')
	            # plt.plot(ptsOut[:,0], ptsOut[:,1], 'o',color='blue')
	            # if ptsObs.size > 0:
	            # 	plt.plot(ptsObs[:,0], ptsObs[:,1], 'o',color='red')
	            # plt.plot(orderedPoses[:,0], orderedPoses[:,1], 'o',color='yellow')
	            # plt.plot(orderedPoses[self.iNextPose,0], orderedPoses[self.iNextPose,1], 'o',color='green')
	            # plt.axis('equal')
	            # plt.xlim((-1,5))
	            # plt.ylim((-1,6))
	            # plt.title('ordered poses')
	            # plt.show()
	            # find starting index on path

	         #call function to compute commands based on self.pose,self.path,self.iNextPose
            
            if self.publishNextPose:
            	print("PUBLISH NEXT POSE")
            	msg = nextPose() 
            	msg.nextPose = self.path[:,self.iNextPose]
            	print(msg.nextPose)
            	self.nextPose_pub.publish(msg)
            	self.publishNextPose = 0

            	# self.iNextPose = (self.iNextPose+1)%self.path.shape[1] #put me in command publisher
            	# print(self.iNextPose)

        elif (self.state=='task'):
            print('state: task')
       	    if self.getNewPath:
	            orderedPoses,temp = generateLoop(self.nBisections,self.minDistance,self.direction,
	            								 self.map[range(0,3),:],self.map[range(3,8),:],self.objectPoints)
	            self.iNextPose = findClosestPointIndex(orderedPoses,self.pose)
	            self.path = np.transpose(orderedPoses)
	            print('New object: generated new path')
	            self.getNewPath = False

	            # ptsIn  = self.map[range(0,3),:]
	            # ptsOut = self.map[range(3,8),:]
	            # ptsObs = self.objectPoints
	            # # plt.figure()
	            # plt.plot(ptsIn[:,0], ptsIn[:,1], 'o',color='blue')
	            # plt.plot(ptsOut[:,0], ptsOut[:,1], 'o',color='blue')
	            # if ptsObs.size > 0:
	            # 	plt.plot(ptsObs[:,0], ptsObs[:,1], 'o',color='red')
	            # plt.plot(orderedPoses[:,0], orderedPoses[:,1], 'o',color='yellow')
	            # plt.plot(orderedPoses[self.iNextPose,0], orderedPoses[self.iNextPose,1], 'o',color='green')
	            # plt.axis('equal')
	            # plt.xlim((-1,5))
	            # plt.ylim((-1,6))
	            # plt.title('ordered poses')
	            # plt.show()
	            #find starting index on path


	            #DO TASK HERE
	            # if (self.objectLabels[-1] == 10):
	            # 	task = 'inside'
	            # elif (self.objectLabels[-1] == 11):
	            # 	task = 'outside'
	            # iLeavePt1,iLeavePt2,dropOffPt = generateTaskPath(self.objectPoints[-1,:],self.map[[0,4,8],:],self.map[[1,2,3,5,6,7],:],
	            # 		            							 self.objectPoints,orderedPoses,task,self.minDistance)
	            # # iLeavePt1,iLeavePt2,dropOffPt = generateTaskPath(self.objectPoints[-1,:],self.map[range(0,3),:],self.map[range(3,9),:],
	            # 											     # self.objectPoints,orderedPoses,task,self.minDistance)

	            # #are these variables needed?
	            # self.objectClassified = False
	            # self.objectPickedUp   = False
	            # self.objectDroppedOff = False

	            # print(self.path.shape)

            #WHEN TASK IS COMPLETE
            self.state = 'explore'
       		#adjust path now that object gone
       		# print('Completed task')
       		# delete(self.objectPoints, (-1), axis=0)
       		# self.getNewPath = True

#            orderedPoses = generateLoop(map,objects,goal)
           # taskpoints = generateTaskPath(map,objects,goal)
#            if currentPose is classifyPoint:
#                #publish commands to do classification
#                classified = 1
#                objectType = something
#            if classified and ball/bear:
#                traverse orderedPoses until reach leavePt1
#                publish commands to pick up object, return to path
#                traverse orderedPoses until reach leavePt2
#                publish commands to drop off object
#                if 1 task done, self.state = 'explore'
#                if 2 tasks done, self.state = 'done'
#        elif (self.state=='done'):
#            play sound

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    rospy.init_node('state_machine', anonymous=True)
    settings = termios.tcgetattr(sys.stdin)
    print('state machine started')
    main = Main()

    try:
        # while True:
            # main.run()
            # sleep(0.01)
            # key = getKey()
            # if (key == 'q'):
            #     break
            
            # sleep(0.01)

        rospy.spin()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    except KeyboardInterrupt:
        print("Shutting down")
