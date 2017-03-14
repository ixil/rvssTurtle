#!/usr/bin/env python2

import os
import rospy
import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import inv
from collections import defaultdict

#Message types
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rvss_workshop.msg import cylDataArray
from rvss_workshop.msg import cylMsg
from rvss_workshop.msg import kalmanLocalizationPose
from rvss_workshop.msg import startstop

#Functions
from utils import Relative2AbsolutePose
from utils import Relative2AbsoluteXY 
from utils import Absolute2RelativeXY
from utils import pi2pi

# Kalman Filter Class contains the predition and perception update.        
class KalmanFilter:

    def __init__ (self, mean, covariance):
        
        self.stateMean = mean 
        self.stateCovariance = covariance 
        self.seenLandmarks = []
    
    # Expected dimesion of the state mean.     
    def expected_dim(self):
        return 3

    # Set state mean
    def setStateMean (self, mean):
        # Check the state mean remains a coloumn vector
        if mean.shape[1] != 1:
            raise ValueError
        if mean.shape[0] != self.expected_dim():
            raise ValueError
        # If the new mean is the correct dimesion, update it
        self.stateMean = mean
        
    def setRobotPose(self, new_pose):
        # Check the robot pose is a 3x1 column vector
        if new_pose.shape != (3,1):
            raise ValueError
        # If the new robot pose is the correct dimesion, update it
        self.stateMean[0,0] = new_pose[0,0] # robot X
        self.stateMean[1,0] = new_pose[1,0] # robot Y
        self.stateMean[2,0] = new_pose[2,0] # robot Theta
    
    def getStateMean (self):
        return np.copy(self.stateMean)
        
    def getRobotPose(self):
        # Use the state mean to keep track of the robot position
        return np.copy(self.stateMean[0:3,:])
    
    def setStateCovariance (self, covariance):
        # Check the covariance matrix remains square and the correct dimesion
        if covariance.shape[0] != self.expected_dim() or covariance.shape[1] != self.expected_dim():
            raise ValueError
        # If the new covariance matrix is the correct dimesion, update it
        self.stateCovariance = covariance
        
    def setRobotCovariance(self, new_cov):
        # Check the robot covariance is the correct dimension
        if new_cov.shape != (3,3):
            raise ValueError
        # If the new robot covariance is the correct dimesion, update it
        self.stateCovariance[0:3,0:3] = new_cov
    
    def getStateCovariance (self):
        return np.copy(self.stateCovariance)
        
    def getRobotCovariance(self):
        # Use the state covariance to keep track of the robot covariance
        return np.copy(self.stateCovariance[0:3,0:3])
         
    def predict (self,motion, motionCovariance):
        # Check correct dimensions of the motion and the motion covariance
        if motion.shape != (3,1):
            raise ValueError
        if motionCovariance.shape != (3,3):
            raise ValueError
        
        # Predict state mean using R2Apose
        [new_pose, F, W] = Relative2AbsolutePose(self.getRobotPose(), motion)

        # Predict state covariance      
        new_cov = np.dot(np.dot(F, self.getRobotCovariance()), np.transpose(F)) + np.dot(np.dot(W, motionCovariance), np.transpose(W))
        
        # Update the robot position and covariance in the state mean and covariance
        self.setRobotPose(new_pose)
        self.setRobotCovariance(new_cov)
        
    def update(self, measurement, measurementCovariance):
        
        stateMean = self.getStateMean()
        stateCovariance = self.getStateCovariance()
        
        label = measurement[2] # Measurement format (X, Y, Label)
        # Calculate the observed measurment from sensor reading
	observedMeasurement = np.array([ [measurement[0]],[measurement[1]] ])
	
        # Calculate the expected measurement from state mean + A2Rxy
        mapGT = np.array(localization.Map)
        landmarkXY = np.array([mapGT[label-1]]).transpose()
        
        # Calculate the expected measurement from state mean + A2Rxy
	[expectedMeasurement,Hr,Hl] = Absolute2RelativeXY(self.getRobotPose(),landmarkXY)
        
        # Calculate the Innovation
	y = observedMeasurement - expectedMeasurement
	
	S = np.dot(np.dot(Hr,self.getStateCovariance()),np.transpose(Hr)) +  np.dot(np.dot(Hl, measurementCovariance),np.transpose(Hl))
	
	try:
            # Calculate the Kalman Gain
            Kt = np.dot(np.dot(self.getStateCovariance(),np.transpose(Hr)),inv(S))		
        except np.linalg.LinAlgError:
            print "S is not invertible"
            return
        
        # Compute posterior mean
        posteriorStateMean = self.getStateMean()+np.array(np.dot(Kt,y))
        
        # Compute posterior covariance
        posteriorStateCovariance = np.dot((np.eye(len(self.getStateMean())) - np.dot(Kt,Hr)),self.getStateCovariance())
        
        # check theta robot is a valid theta in the range [-pi, pi]
        posteriorStateMean[2,0] = pi2pi(posteriorStateMean[2,0])
        
        # POST PROCESSING - IF the update is too big discard the update
        norm = np.linalg.norm(posteriorStateMean[0:2] - self.getStateMean()[0:2])
        if norm > 1: # If the update of the robot abs position is more than 1m, discard
            print "Update discarded.  Norm of state difference is: ", norm
        else:
            # set posterior state mean
            self.setStateMean(posteriorStateMean)
            # set posterior state covariance
            self.setStateCovariance(posteriorStateCovariance)
    
class Localization:
        
    def callbackOdometryMotion(self, msg):

        # Use the IMU measurement for angular velocity instead of the odometry
        ang_z = (self.last_angular_z)
        
        # Construct the velocity measurement vector
        ddt = np.array([[msg.twist.twist.linear.x], [0], [ang_z]])
        time = msg.header.stamp.secs + 1.0E-9* msg.header.stamp.nsecs
        
        # For the first measurement, we can't calculate a time step
        if(self.last_time == 0):
            self.last_time = time
            return
        time_step = time - self.last_time
        self.last_time = time
        
        # Calculate the motion command and set a constant covariance
        motion_command = ddt*time_step

        motion_cov = np.zeros((3,3))
        motion_cov[0,0] = 0.001
        motion_cov[1,1] = 0.0002   
        motion_cov[2,2] = 0.0005   

        # Call Kalman filter class to execute a prediction
        self.KF.predict(motion_command, motion_cov)
        
        # Publish the current state for the plot
        self.publish_state()
        
    def callbackLandmarkMeasurement(self, data):
        for i in range(0,len(data.cylinders)):
            # Align landmark measurement frame with robot frame. Measurements are converted from computer vision to robot frame
            dx = data.cylinders[i].Zrobot 
            dy = -data.cylinders[i].Xrobot 
            label = data.cylinders[i].label 
            
            # POST PROCESSING:
            # Measurement reading outside field, discard
            # Measurment taken when ||angular veocity|| of robot is < 0.5
            if dy < - 2 or dy > 2 or dx > 6  or self.last_angular_z > 0.5 or self.last_angular_z < -0.5: 
                print 'Measurement Discarded: ', dx, ', ', dy, ', ', label
                continue
            
            # Create Measruement vector
            measurement = [dx, dy, label]
            
            # Use constant measurement covariance
            covariance = np.array([[0.15, 0],[0, 0.08]])

            # Call Kalman Filter to execute an update
            self.KF.update(measurement, covariance)  
                      
    def callbackImu(self, data):
        self.last_angular_z = data.angular_velocity.z
        
    # The startStop topic re-zeros all robot data
    def callbackstartstop(self, data):
        if data.startstop == 1:
            self.zeroState()
    
    # Publish state data for the plot node, txt creater node and cylinder counter node
    def publish_state(self):
        msg = kalmanLocalizationPose() 
        msg.stateMean = self.KF.getStateMean().flatten()
        msg.stateCovariance = self.KF.getStateCovariance().flatten()
        self.pose_pub.publish(msg)
        
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def zeroState(self):
        # Initialise robot to 0,0,0
        robot_pose = np.array([[0.0],[0.0],[0.0]])
        robot_covariance = np.array([[0.01, 0, 0],[0,0.01,0],[0,0,0.01]])
        
        # Initialise the IMU measurement and last message time
        self.last_angular_z = 0.0
        self.last_time = 0
        
        #Initialise the state mean and covariance with the robot's pose and cov
        self.KF = KalmanFilter(robot_pose, robot_covariance)
        
    def __init__(self, map): 
        # Initialise robot to 0,0,0
        self.zeroState()
        self.pose_pub = rospy.Publisher("/pose", kalmanLocalizationPose, queue_size=5)
        self.Map = map
       
        # Subscribe to relevant topics
        self.cyl_sub   = rospy.Subscriber("/cylinderTopic",cylDataArray,self.callbackLandmarkMeasurement)
        self.odom_sub  = rospy.Subscriber("/odom", Odometry, self.callbackOdometryMotion)
        self.imu_sub   = rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, self.callbackImu)
        self.startstop = rospy.Subscriber("/startstop", startstop, self.callbackstartstop)
        
      
if __name__ == '__main__':
    rospy.init_node('listener')
    home = os.getcwd()
    f = open(home[0:-5]+'/RVSS_ws/src/rvss_workshop/scripts/GT.txt','r')
    map = np.loadtxt(f)
    localization = Localization(map)
    print('Localization Started...')
    rospy.spin()
