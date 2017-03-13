#!/usr/bin/env python2

import rospy
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np
from rvss_workshop.msg import kalmanState
from rvss_workshop.msg import startstop
from rvss_workshop.msg import cylDataArray
from rvss_workshop.msg import cylMsg
import os

# Turn interactive plotting on.
plt.ion()

class SLAM_PLOT():

    def __init__(self):
        # Subscribe to relevant topics
        self.state_sub = rospy.Subscriber("/state", kalmanState, self.callbackState)
        self.startstop = rospy.Subscriber("/startstop", startstop, self.callbackstartstop)
        self.cyl_sub = rospy.Subscriber("/cylinderTopic", cylDataArray,self.callbackCylinder)
        
        # Open Figure
        self.fig, self.ax = plt.subplots()
        self.groundTruth, = self.ax.plot([],[],'o')

        # Create the axes for plotting
        self.robot, = self.ax.plot([],[], 'ko', label="Robot Position")
        self.pastRobot, = self.ax.plot([],[],'y.', label="Robot Trajectory")
        self.cylinders, = self.ax.plot([],[], 'ro', label="Cylinder Position")
        self.cylinder_observations, =self.ax.plot([],[],'g.', label="Cylinder Observations")
        self.robotDirection, = self.ax.plot([],[],'r-', label="Robot Direction")
        self.robotDirectionUncertaintyLeft, = self.ax.plot([],[],'y-', label="Direction Uncertainty")
        self.robotDirectionUncertaintyRight, = self.ax.plot([],[],'y-')
        
        # Auto resize the plot figure
        self.ax.set_autoscaley_on(True)
        self.ax.set_autoscalex_on(True)

        # Grid on
        self.ax.grid()
        
        # Init class variables
        self.lastMessage = kalmanState()
        self.lastMessage.stateMean = [0,0,0]
        self.landmarkAnnotations = []
        self.landmarkEllipses = []
        self.robotEllipse = None
        self.cov = np.array([[0,0,0],[0,0,0],[0,0,0]])
        
        # Plot Ground Truth
        self.plotGroundTruth()
        
        #Draw legend
        self.ax.legend(fontsize='xx-small', numpoints=1, loc=4)
        
        # Continually replot every 0.1 seconds
        while True:
            self.plotCylinders()
            self.plotRobot()
            self.drawNow()
            plt.pause(0.1)
            
   
   # Save the state and covariance matrices 
    def callbackState(self, data):
        self.lastMessage = data
        dim = np.sqrt(len(data.stateCovariance))
        self.cov = np.reshape(data.stateCovariance, (dim,dim))
        
    #Reset the plot
    def callbackstartstop(self, data):
        if data.startstop == 1:
            self.pastRobot.set_data([],[])
            self.cylinder_observations.set_data([],[])
            self.cylinders.set_data([],[])
            self.robotDirection.set_data([],[])
            self.robotDirectionUncertaintyLeft.set_data([],[])
            self.robotDirectionUncertaintyRight.set_data([],[])
            self.robot.set_data([],[])
            # init stateMean
            self.lastMessage = kalmanState()
            self.lastMessage.stateMean = [0,0,0]
    
    # Plot the cylinder observations
    def callbackCylinder(self, data):
        for i in range(0,len(data.cylinders)):
            #Convert to the right coordinates
            dx = data.cylinders[i].Zrobot
            dy = -data.cylinders[i].Xrobot
            if dy < - 2 or dy > 2 or dx > 6:
                continue
            
            #Convert to absolute coordinates
            rx = self.lastMessage.stateMean[0]
            ry = self.lastMessage.stateMean[1]
            rt = self.lastMessage.stateMean[2]
            
            x = rx + dx * np.cos(rt) - dy * np.sin(rt)
            y = ry + dx * np.sin(rt) + dy * np.cos(rt)
            
            # Append to the cylinder plots
            self.cylinder_observations.set_xdata(np.append(self.cylinder_observations.get_xdata(), [x]))
            self.cylinder_observations.set_ydata(np.append(self.cylinder_observations.get_ydata(), [y]))
    
    def drawNow(self):
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
    def plotGroundTruth(self):
        #Read the GT file and plot each point
        home = os.getcwd()
        f = open(home+'/RVSS_ws/src/rvss_workshop/scripts/GT.txt','r')
    	gt = np.loadtxt(f)
           
        # Add labels to each point
        self.ax.plot(gt[:,0], gt[:,1],'bx', label="Cylinder Ground Truth")
        for i in range(0,len(gt)):
            self.ax.annotate(str(i+1), xy = (gt[i,0], gt[i,1]))
    
    # plot each cylinder and the surrounding covariance ellipse
    def plotCylinders(self):
        #delete all the existing labels and ellipses
        for a in self.landmarkAnnotations:
            a.remove()
        for e in self.landmarkEllipses:
            e.remove()
        self.landmarkAnnotations = []
        self.landmarkEllipses = []
    
        if not self.lastMessage:
            return
        state = np.array(self.lastMessage.stateMean)
        x = state[3::2]
        y = state[4::2]
        self.cylinders.set_data(x, y)
        
        for i in range(0,len(self.lastMessage.seenLandmarks)):
            a = self.ax.annotate(str(int(self.lastMessage.seenLandmarks[i])), xy = (x[i],y[i]))
            self.landmarkAnnotations.append(a)
            e = Ellipse(xy=(x[i],y[i]), width=np.sqrt(self.cov[3+i*2, 3+i*2]), height=np.sqrt(self.cov[4+i*2, 4+i*2]), alpha=0.5)
            self.ax.add_artist(e)
            self.landmarkEllipses.append(e)

    #Plot the robot position include an ellipse to indicate uncertainty
    def plotRobot(self):
        if self.robotEllipse:
            self.robotEllipse.remove()
            self.robotEllipse = None
        
        if not self.lastMessage:
            return
        
        x = self.lastMessage.stateMean[0]
        y = self.lastMessage.stateMean[1]
        t = self.lastMessage.stateMean[2]
        
        self.robot.set_data([x], [y])
        
        #Plot a line to indicate direction and two smaller lines to indicate uncertainty in direction
        self.robotDirection.set_data([x, x + np.cos(t)], [y, y + np.sin(t)])
        sigma_t = np.sqrt(self.cov[2,2])
        self.robotDirectionUncertaintyLeft.set_data( [x, x + 0.5*np.cos(t-sigma_t)], [y, y + 0.5*np.sin(t-sigma_t)])
        self.robotDirectionUncertaintyRight.set_data([x, x + 0.5*np.cos(t+sigma_t)], [y, y + 0.5*np.sin(t+sigma_t)])   

        self.robotEllipse = Ellipse(xy = (x,y), width=np.sqrt(self.cov[0,0]), height=np.sqrt(self.cov[1,1]), fill=False)
        self.ax.add_artist(self.robotEllipse)
        
        self.pastRobot.set_data(np.insert(self.pastRobot.get_xdata(), 0, x), np.insert(self.pastRobot.get_ydata(), 0, y))


if __name__ == '__main__':
    print('Plot Node Initialized')
    rospy.init_node('SLAM_plot')
    s = SLAM_PLOT()
    rospy.spin()
