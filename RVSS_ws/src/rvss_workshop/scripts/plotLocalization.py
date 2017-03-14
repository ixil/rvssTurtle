#!/usr/bin/env python2

import rospy
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np
from rvss_workshop.msg import kalmanLocalizationPose
from rvss_workshop.msg import startstop
import os

# Turn interactive plotting on.
plt.ion()

class SLAM_PLOT():

    def __init__(self):
        # Subscribe to relevant topics
        self.pose_sub = rospy.Subscriber("/pose", kalmanLocalizationPose, self.callbackPose)
        self.startstop = rospy.Subscriber("/startstop", startstop, self.callbackstartstop)
        
        # Open Figure
        self.fig, self.ax = plt.subplots()
        self.groundTruth, = self.ax.plot([],[],'o')

        # Create the axes for plotting
        self.robot, = self.ax.plot([],[], 'ko', label="Robot Position")
        self.pastRobot, = self.ax.plot([],[],'y.', label="Robot Trajectory")
        self.robotDirection, = self.ax.plot([],[],'r-', label="Robot Direction")
        self.robotDirectionUncertaintyLeft, = self.ax.plot([],[],'y-', label="Direction Uncertainty")
        self.robotDirectionUncertaintyRight, = self.ax.plot([],[],'y-')
        
        # Auto resize the plot figure
        self.ax.set_autoscaley_on(True)
        self.ax.set_autoscalex_on(True)

        # Grid on
        self.ax.grid()
        
        # Init class variables
        self.lastMessage = False
        self.robotEllipse = None
        
        #Draw legend
        self.ax.legend(fontsize='xx-small', numpoints=1, loc=4)
        
        # Continually replot every 0.1 seconds
        while True:
            self.plotRobot()
            self.drawNow()
            plt.pause(0.1)
            
   
   # Save the state and covariance matrices 
    def callbackPose(self, data):
        self.lastMessage = data
        dim = np.sqrt(len(data.stateCovariance))
        self.cov = np.reshape(data.stateCovariance, (dim,dim))
        
    #Reset the plot
    def callbackstartstop(self, data):
        if data.startstop == 1:
            self.pastRobot.set_data([],[])
            self.robotDirection.set_data([],[])
            self.robotDirectionUncertaintyLeft.set_data([],[])
            self.robotDirectionUncertaintyRight.set_data([],[])
            self.robot.set_data([],[])
            self.lastMessage = False
                
    def drawNow(self):
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
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
    rospy.init_node('lab4_plot')
    s = SLAM_PLOT()
    rospy.spin()
