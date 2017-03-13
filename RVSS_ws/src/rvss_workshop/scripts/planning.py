#!/usr/bin/env python2

import matplotlib.pyplot as plt
import matplotlib.tri as tri
import numpy as np
import math
from scipy.spatial import Voronoi, voronoi_plot_2d
from operator import itemgetter
from numpy import linalg as LA
import matplotlib.path as mplPath

from utils import (sortPoints,cart2pol,sign,pointInPolygon,bisectPolygon,
				   getMidPoints,addConnection,pointToLineDistance,lineToLineDistance,distance,
				   intersect,ccw)

#================================================================================
#================================================================================

# Exploration path function

#================================================================================
#================================================================================

def generateLoop(nBisections,minDistance,direction,ptsIn,ptsOut,ptsObs):
	"Generates path from voronoi diagram"
	#Inputs
	#	nBisections - (int) no. times to bisect hexagon sides
	#	minDistance - (float) closest turtlebot centre can be from obstacles
	#	direction   - (string) 'anticlockwise'/'clockwise': direction to move in

	#Outputs
	#	orderedPoses   - (array) nPoses x 3, each row = [x,y,theta] in global coords
	#				   - ordered array of poses forming safest path for robot to move in
	#	orderedQuivers - (array) nPoses x 4, each row = [x,y,dX,dY] in global coords
	#	               - [dX dY] is vector to next pose. this variable is best used for 
	#				   - plotting

	#================================================================================
	# 1. Adjust environment
	#================================================================================

	ptsInOriginal  = ptsIn
	ptsOutOriginal = ptsOut

	#create line obstacles: pairs of points
	ptsOut = sortPoints(ptsOut)
	nPtsOut = len(ptsOut)
	nPtsIn  = len(ptsIn)
	linesObs = []
	for i in range(0,nPtsIn):
		linesObs.append(np.vstack((ptsIn[i,:],ptsIn[(i+1)%nPtsIn,:])))
	for i in range(0,nPtsOut):
		linesObs.append(np.vstack((ptsOut[i,:],ptsOut[(i+1)%nPtsOut,:])))

	#bisect ptsOut
	for i in range(0,nBisections):
		ptsOut = bisectPolygon(ptsOut)
		
	#concatenate points
	if ptsObs.size > 0:
		pts = np.vstack((ptsIn,ptsOut,ptsObs))
	else:
		pts = np.vstack((ptsIn,ptsOut))

	#================================================================================
	# 2. Voronoi diagram
	#================================================================================
	vor = Voronoi(pts)
	nVertices = len(vor.vertices)
	for i in range(0,nVertices):
		if pointInPolygon(ptsInOriginal,vor.vertices[i,:]):
			iInnerVertex = i
	vertices = vor.vertices
	centrePoint = vertices[iInnerVertex,:]

	#form list of lists, which vertices are connected to one another
	connections = [[-1]]*nVertices
	nConnections = len(vor.ridge_vertices)
	for i in range(0,nConnections):
		pair = vor.ridge_vertices[i]
		v1 = pair[0]
		v2 = pair[1]
		if (v1 >= 0) & (v2 >= 0):
			connections[v1] = addConnection(connections[v1],v2)
			connections[v2] = addConnection(connections[v2],v1)

	#identify vertices for removal/to be ignored
	remove = [iInnerVertex]
	connections[iInnerVertex] = []
	done = 0
	while (not done):
		flag = 1
		for i in range(0,nVertices):
			if (len(connections[i])==1) & (i not in remove):
				remove.append(i)
				connections[i] = []
				flag = 0	
		for i in range(0,nVertices):
			connections[i] = [x for x in connections[i] if x not in remove]
		if (flag==1):
			done = 1

	#================================================================================
	# 3. Generate optimum path
	#================================================================================
	#from connections, generate list of segments
	segments = []
	nodes = []
	for i in range(0,nVertices):
		if (len(connections[i]) > 2):
			nodes.append(i)
		for j in range(0,len(connections[i])):
			if (connections[i][j] > i):
				segments.append([i,connections[i][j]])
	nSegments = len(segments)
	segments = sorted(segments,key=itemgetter(0,1))
	distances = np.asarray([float('inf')]*nSegments) #initialise as inf

	#no obstacles
	if (len(nodes)==0):
		points = np.delete(vertices,remove,axis=0)
		orderedPoints = sortPoints(points)
		if (direction=='clockwise'):
			orderedPoints = orderedPoints[::-1,:]
		nPoses = orderedPoints.shape[0]
		orderedPoses = np.hstack((orderedPoints,np.zeros((nPoses,1))))
		orderedQuivers = np.hstack((orderedPoints,np.zeros((nPoses,2))))
		for i in range(0,nPoses):
			dY = orderedPoints[(i+1)%nPoses,1] - orderedPoints[i,1]
			dX = orderedPoints[(i+1)%nPoses,0] - orderedPoints[i,0]
			orderedPoses[i,2] = np.arctan2(dY,dX)
			orderedQuivers[i,2] = dX
			orderedQuivers[i,3] = dY
		return orderedPoses,orderedQuivers

	#compute min distance to obstacle for each segment
	for i in range(0,nSegments):
		#ends of segment i
		v1 = vertices[segments[i][0],:]
		v2 = vertices[segments[i][1],:]
		for j in range(0,len(ptsObs)):	#point obstacles
			d  = pointToLineDistance(v1,v2,ptsObs[j,:])
			distances[i] = min(distances[i],d)
		for j in range(0,len(linesObs)): #line obstacles
			d  = lineToLineDistance(linesObs[j][0,:],linesObs[j][1,:],v1,v2)
			distances[i] = min(distances[i],d)

	#identify bad segments, remove bad segments from connections
	iBadSegments = (distances<minDistance).nonzero()[0]
	badSegments = [segments[i] for i in iBadSegments]
	for i in range(0,len(badSegments)): #remove from connections
		v1 = badSegments[i][0]
		v2 = badSegments[i][1]
		connections[v1].remove(v2)
		connections[v2].remove(v1)

	#create branches as lists of points
	#	start at each node, traverse connections until another node reached -> create branch
	branches = []
	for i in range(0,len(nodes)):
	 	for j in range(0,len(connections[nodes[i]])):
	 		currentPoint = connections[nodes[i]][j]
			branch = [nodes[i],currentPoint]
			nodeReached = 0
			#keep adding points to branch until reach node or dead end
			if (currentPoint in nodes) & (branch[0] < branch[-1]):
				branches.append(branch)
			else:
				while (not nodeReached):
					if (len(connections[currentPoint]) == 2):
						currentPoint = list(set(connections[currentPoint])-set(branch))[0]
						branch.append(currentPoint)
						if (currentPoint in nodes):
							nodeReached = 1					
					else:
						nodeReached = 1	
				if (currentPoint in nodes) & (branch[0] < branch[-1]):
					branches.append(branch)

	#switch branch directions based on angles
	startNodes = []
	endNodes   = []
	for i in range(0,len(branches)):
	# n = 5
	# for i in range(n,n+1):
		node1 = branches[i][0]
		node2 = branches[i][-1]
		pStart = vertices[node1,:]
		pEnd   = vertices[node2,:]
		#reverse 2 node branches if clockwise
		if (len(branches[i]) == 2):
			startToEndCCW = ccw(centrePoint,pStart,pEnd)
			if (not startToEndCCW):
				branches[i] = branches[i][::-1]	
		#reverse > 2 node branches if clockwise		
		elif (len(branches[i]) > 2): 
			middle = branches[i][len(branches[i])/2]
			pMiddle = vertices[middle,:] 
			startToMiddleCCW = ccw(centrePoint,pStart,pMiddle)
			middleToEndCCW   = ccw(centrePoint,pMiddle,pEnd)
			if not(startToMiddleCCW and middleToEndCCW):
				branches[i] = branches[i][::-1]	
		#reverse all if clockwise desired
		if (direction == 'clockwise'):
			branches[i] = branches[i][::-1]
		#store start and end nodes
		startNodes.append(branches[i][0])
		endNodes.append(branches[i][-1])

	#sort by endpoints
	#branches = sorted(branches,key=itemgetter(0,-1))

	#remove dead ends (nodes only appearing in endNodes)
	deadEnds = list(set(endNodes)-set(startNodes))
	while (len(deadEnds) > 0): #repeat until no dead ends
		nodes = [x for x in nodes if x not in deadEnds]
		deadBranches = []
		for i in range(0,len(branches)):
			if endNodes[i] in deadEnds:
				deadBranches.append(i)
		startNodes = [i for j, i in enumerate(startNodes) if j not in deadBranches]
		endNodes = [i for j, i in enumerate(endNodes) if j not in deadBranches]
		branches = [i for j, i in enumerate(branches) if j not in deadBranches]
		deadEnds = list(set(endNodes)-set(startNodes))
	if (len(branches)==0):
		print('Error: all branches removed')

	#get min distance to obstacle of each branch
	if (len(branches) > 0):
		branchDistances = np.asarray([float('inf')]*len(branches)) #initialise as inf
		for i in range(0,len(branches)):
			for j in range(0,len(branches[i])-1):
				p1 = branches[i][j]
				p2 = branches[i][j+1]
				#find segment connected p1,p2
				iSegment = segments.index(sorted([p1,p2],key=int))
				branchDistances[i] = min(branchDistances[i],distances[iSegment])

	#create ordered list of branches
	currentNode = nodes[0]
	nodeSequence = [currentNode]
	branchSequence = []
	done = 0
	while (not done):
		#find currentNode in startNodes
		currentBranches = [i for i, x in enumerate(startNodes) if x == currentNode]
		currentDistances = branchDistances[currentBranches]
		branchIndex = currentBranches[np.argmax(currentDistances)]
		branchSequence.append(branchIndex)
		currentNode = branches[branchIndex][-1]
		if (nodeSequence[0]==currentNode):
			done = 1
		elif (len(nodeSequence) > 2*len(nodes)):
			done = 1
			print('Error: could not complete cycle')
		else:
			nodeSequence.append(currentNode)

	# branches sequence -> ordered points -> ordered poses
	pointSequence = []
	for i in range(0,len(branchSequence)):
		pointSequence = pointSequence + branches[branchSequence[i]]
		del pointSequence[-1] #remove duplicate
	orderedPoints = vertices[pointSequence,:]
	nPoses = orderedPoints.shape[0]
	orderedPoses = np.hstack((orderedPoints,np.zeros((nPoses,1))))
	orderedQuivers = np.hstack((orderedPoints,np.zeros((nPoses,2))))
	for i in range(0,nPoses):
		dY = orderedPoints[(i+1)%nPoses,1] - orderedPoints[i,1]
		dX = orderedPoints[(i+1)%nPoses,0] - orderedPoints[i,0]
		orderedPoses[i,2] = np.arctan2(dY,dX)
		orderedQuivers[i,2] = dX
		orderedQuivers[i,3] = dY
	return orderedPoses,orderedQuivers



#================================================================================
#================================================================================

# Task path function

#================================================================================
#================================================================================
def generateTaskPath(ptGoal,ptsIn,ptsOut,ptsObs,orderedPoses,task,safeDistance):
	"Given a goal point, outputs points necessary to complete pushing task"
	#Inputs
	#	ptGoal - (array), position of object to perform task with
	#	ptsIn  - (array), positions of cylinders forming triangle boundary
	#	ptsOut - (array), positions of cylinders forming hexagon boundary
	#	ptsObs - (array), positions of obstacles
	#	orderedPoses - (array), poses forming safe path
	#	task         - (string), 'inside'/'outside' - where to put object
	#	safeDistance - (float), min distance turtlebot centre can be from any obstacle

	#Outputs
	#	iLeavePt1 - (int), index of closest point on safe path to object
	#	iLeavePt2 - (int), index of point on safe path to leave and drop off object
	#	dropOffPt - (array), point to leave object at	

	#find closest point in loop to goal
	minPoint = 0
	minDistance = 'inf'
	for i in range(0,orderedPoses.shape[0]):
		d = distance(ptGoal,orderedPoses[i,[0,1]])
		if d < minDistance:
			minPoint = i
			minDistance = d

	#starting at minpoint, find closest goal point on inside/outside midpoints
	#see if segment is safe
	#if so, add to taskPoses
	ptsOut = sortPoints(ptsOut)
	nPtsOut = len(ptsOut)
	nPtsIn  = len(ptsIn)
	ptsObs = np.vstack((ptsObs,ptsIn,ptsOut)) #avoid all cylinders
	linesObs = []
	if (task=='inside'):
		midPoints = getMidPoints(ptsIn)
		for i in range(0,nPtsOut):
			linesObs.append(np.vstack((ptsOut[i,:],ptsOut[(i+1)%nPtsOut,:])))
	elif (task=='outside'):
		midPoints = getMidPoints(ptsOut)
		for i in range(0,nPtsIn):
			linesObs.append(np.vstack((ptsIn[i,:],ptsIn[(i+1)%nPtsIn,:])))

	done = 0
	currentPoint = minPoint
	while (not done):
		#compute distance from currentPoint to midPoints
		minDistance = 'inf'
		for i in range(0,midPoints.shape[0]):
			d = distance(orderedPoses[currentPoint,[0,1]],midPoints[i,:])
			if d < minDistance:
				minDistance = d
				closestPoint = midPoints[i,:]

		p1 = orderedPoses[currentPoint,[0,1]]
		p2 = closestPoint

		#check if path to drop off point is blocked
		goodSegment = True
		for i in range(0,len(ptsObs)):	#point obstacles
			d  = pointToLineDistance(p1,p2,ptsObs[i,:])
			if d < safeDistance:
				goodSegment = False
				break
		if goodSegment:		
			for i in range(0,len(linesObs)): #line obstacles
				d  = lineToLineDistance(linesObs[i][0,:],linesObs[i][1,:],p1,p2)
				if d < safeDistance:
					goodSegment = False
					break
		if goodSegment:
			done = 1
		else:
			currentPoint = (currentPoint + 1)%orderedPoses.shape[0]

	#assign outputs - this is just for clarity
	iLeavePt1 = minPoint
	iLeavePt2 = currentPoint
	dropOffPt = closestPoint

	return iLeavePt1,iLeavePt2,dropOffPt
