#!/usr/bin/env python2

import matplotlib.pyplot as plt
import matplotlib.tri as tri
import numpy as np
import math
from scipy.spatial import Voronoi, voronoi_plot_2d
from operator import itemgetter
from numpy import linalg as LA
import matplotlib.path as mplPath

# mapping --------------------------------------------------------
def mapping(i):
    dim = 2; # dimension of each landmark
    vec = []  
    for i in range ((i-1)* dim + 1, ((i-1)* dim) + dim + 1):
        vec.append(i)   
    return vec

# RelativeLandmarkPositions --------------------------------------------------------
def RelativeLandmarkPositions(landmark_abs, next_landmark_abs):
    
    '''
    Calculate the relative landmark positions
    Input: absolute coordinate of landmark [x1,y1]
           absolute coordinate of landmark next position [x2,y2]
    Output: relative position [dx, dy]
    '''
    # label is in position [0]
    x1 = float(landmark_abs[1])
    y1 = float(landmark_abs[2])
    x2 = float(next_landmark_abs[1])
    y2 = float(next_landmark_abs[2])
    
    #Calculate the difference of position in world frame
    diff = [x2-x1, y2-y1]
    
    return diff

# Absolute2RelativeXY --------------------------------------------------------
def Absolute2RelativeXY(robot_abs,landmark_abs):
    
    '''
    Express Landmark's Coordinate on Robot Frame
    Input: robot's absolute coordinate [x,y,theta]
           landmarks absolute coordinate [x,y]
    Output: landmarks's relative coordinate with repect to robot frame [x,y] 
    i.e. landmark's measurement from robot
    '''
    
    x1 = robot_abs[0][0]
    y1 = robot_abs[1][0]
    theta1 = robot_abs[2][0]
    x2 = landmark_abs[0]    
    y2 = landmark_abs[1]
    

    #Calculate the difference with respect to world frame
    diff = [[x2-x1],
            [y2-y1],
            [1]]
    
    #R is the transition matrix to robot frame
    R = [[np.cos(-theta1), -np.sin(-theta1), 0],
         [np.sin(-theta1), np.cos(-theta1), 0],
         [0, 0, 1]]
         
    #Calculate Jacobian H1 with respect to X1
    H1 = [[-np.cos(theta1), -np.sin(theta1), -(x2-x1)*np.sin(theta1)+(y2-y1)*np.cos(theta1)],
          [np.sin(theta1), -np.cos(theta1), -(x2-x1)*np.cos(theta1)-(y2-y1)*np.sin(theta1)]]
         
    #Calculate Jacobian H2 with respect to X2
    H2 = [[np.cos(theta1), np.sin(theta1)],
          [-np.sin(theta1), np.cos(theta1)]]
         
    landmark_rel_xy = np.dot(R,diff)
    
    return [[landmark_rel_xy[0][0]],[landmark_rel_xy[1][0]]], H1, H2

# Relative2AbsoluteXY --------------------------------------------------------
def Relative2AbsoluteXY(robot_abs,landmark_meas_xy):

    '''
    Calcute Landmark's absolute coordinate
    Input: robot's absolute coordinate [x,y,theta]
           landmark's measurment with repect to robot frame [x,y]
    Output: landmarks's absolute coordinate  [x,y]
    '''
    
    x1 = robot_abs[0][0]
    y1 = robot_abs[1][0]
    theta1 = robot_abs[2][0]
    x2 = landmark_meas_xy[0]    
    y2 = landmark_meas_xy[1]
    
    landmark_meas = [[x2],
                     [y2],
                     [1]]
    
    #R is the transition matrix to robot frame
    R = [[np.cos(theta1), -np.sin(theta1), 0],
         [np.sin(theta1), np.cos(theta1), 0],
         [0, 0, 1]]
         
    #Calculate Jacobian H1 with respect to X1
    H1 = [[1, 0, -x2*np.sin(theta1)-y2*np.cos(theta1)],
          [0, 1,  x2*np.cos(theta1)-y2*np.sin(theta1)]]
         
    #Calculate Jacobian H2 with respect to X2
    H2 = [[np.cos(theta1), -np.sin(theta1)],
          [np.sin(theta1),  np.cos(theta1)]]
         
    landmark_abs = np.array(np.dot(R,landmark_meas)) + np.array(robot_abs) 
    
    return [[landmark_abs[0][0]],[landmark_abs[1][0]]], H1, H2

# Relative2AbsolutePose --------------------------------------------------------
def Relative2AbsolutePose (robot_abs, u):
    
    '''
    Calculate the robot new pose given previous pose and motion
    Input: absolute coordinate of robot [x1,y1,theta1]
           motion command with respect to robot frame [dx, dy, dtheta]
    Output: absolute coordinate of robot next pose [x2,y2,theta2] 
    '''
    x1 = robot_abs[0][0]
    y1 = robot_abs[1][0]
    theta1 = robot_abs[2][0]
    dx = u[0][0]
    dy = u[1][0]
    dtheta = u[2][0]
    
    #R is the transition matrix of robot frame
    R = [[np.cos(theta1), -np.sin(theta1), 0],
         [np.sin(theta1), np.cos(theta1), 0],
         [0, 0, 1]]
         
    #Calculate Jacobian H1 with respect to X1
    H1 = [[1, 0, -dx*np.sin(theta1)-dy*np.cos(theta1)],
          [0, 1,  dx*np.cos(theta1)-dy*np.sin(theta1)],
          [0, 0, 1]]
         
    #Calculate Jacobian H2 with respect to u
    H2 = [[np.cos(theta1), -np.sin(theta1), 0],
          [np.sin(theta1), np.cos(theta1), 0],
          [0, 0, 1]]
     
    next_robot_abs = np.dot(R,u) + robot_abs
    
    next_robot_abs[2][0] = pi2pi(next_robot_abs[2][0])
    
    return next_robot_abs, H1, H2

# pi2pi --------------------------------------------------------
def pi2pi(angle):
        dp = 2*np.pi
        if (angle<=-dp) or (angle>=dp):
            angle = angle % dp
        if angle>=np.pi:
            angle = angle - dp
        if angle<=-np.pi:
            angle = angle + dp
        return angle

# sortPoints --------------------------------------------------------
def sortPoints(pts):
	"Reorders points by angle in polar coordinates wrt centroid of input points"
	centroid = pts.mean(axis=0)
	ptsRel = pts - centroid
	nPts = len(pts)
	angles = np.zeros(nPts)
	for i in range(0,nPts):
		(r,th) = cart2pol(ptsRel[i,0],ptsRel[i,1])
		angles[i] = th
	ptsSorted = pts[np.argsort(angles),:]
	return ptsSorted

# cart2pol --------------------------------------------------------
def cart2pol(x, y):
	"Converts from 2D cartesian to polar coords"
	r = np.sqrt(x**2 + y**2)
	th = np.arctan2(y, x)
	return(r, th)

# sign --------------------------------------------------------
def sign (p1,p2,p3):
	"Computes barymetric coordinate for triangle intersection"
	return (p1[0] - p3[0])*(p2[1] - p3[1]) - (p2[0] - p3[0])*(p1[1] - p3[1])

# pointInPolygon --------------------------------------------------------
def pointInPolygon(polygon,pt):
	"Determines if pt inside polygon"
	outPath = mplPath.Path(polygon)
	return outPath.contains_point((pt[0], pt[1]))

# bisectPolygon --------------------------------------------------------
def bisectPolygon(pts):
	"pts forms polygon. points are added to pts by bisecting each side."
	bisections = getMidPoints(pts)
	pts = np.vstack((pts,bisections))
	pts = sortPoints(pts)
	return pts

# getMidPoints --------------------------------------------------------
def getMidPoints(pts):
	pts = sortPoints(pts) #reorder by polar coord angle
	bisections = np.zeros(pts.shape)
	nPts = len(pts)
	for i in range(0,nPts):
		bisections[i,:] = (pts[[i,(i+1)%nPts],:]).mean(axis=0)
	return bisections

# addConnection --------------------------------------------------------
def addConnection(vConnections,newV):
	"Appends vertex to list, or replaces if list contains -1 only"
	if (vConnections[0]==-1):
		vConnections = [newV]
	else:
		vConnections.append(newV)
	return vConnections

# pointToLineDistance --------------------------------------------------------
def pointToLineDistance(v1,v2,pt):
	"Computes min distance from pt to line segment formed by v1 & v2"
	
	lineVec = v2 - v1
	pntVec  = pt - v1
	lineLen = LA.norm(lineVec)
	
	lineUnitVec = lineVec/lineLen
	pntVecScaled = pntVec/lineLen

	t = np.dot(lineUnitVec,pntVecScaled)
	if t < 0:	
		t = 0
	elif t > 1:
		t = 1
	nearest = t*lineVec
	return distance(nearest,pntVec)

# lineToLineDistance --------------------------------------------------------
def lineToLineDistance(v1,v2,v3,v4):
	if intersect(v1,v2,v3,v4):
		return 0
	else:
		return min(pointToLineDistance(v1,v2,v3),pointToLineDistance(v1,v2,v4))

# distance --------------------------------------------------------
def distance(p1,p2):
	return LA.norm(p1-p2)

# intersect --------------------------------------------------------
# Return true if line segments AB and CD intersect
def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

# ccw --------------------------------------------------------
def ccw(A,B,C):#check if points in counterclockwise order
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

# findClosestPointIndex --------------------------------------------------------
def findClosestPointIndex(waypoints,currentPose):
	iClosestPoint = 0
	minDistance = float('inf')
	for i in range(len(waypoints)):
		d = np.sqrt((waypoints[i,0]-currentPose[0][0])**2 + (waypoints[i,1]-currentPose[1][0])**2)
		if d < minDistance:
			iClosestPoint = i
			minDistance = d
	return iClosestPoint
