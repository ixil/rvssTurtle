#!/usr/bin/env python2

import numpy as np
from utils import RelativeLandmarkPositions 
from utils import pi2pi

'''
Calculate the error between GT data and SLAM obtained  data
Input: - solution file containing:
       - absolute coordinates of landmark positions
       - GT file containing:
       - GT absolute coordinates of landmark positions
       - Assume all cylinders are arranged
Output: error = Relative GT landmark positions - Relative estimated landmark positions
'''

def ErrorFunction (solutionFile, GTFile):
    
     landmark = []
     landmark_GT = []
     if solutionFile.endswith('.csv') and GTFile.endswith('.csv'):
         with open(solutionFile, 'rb') as csvfile:
		 spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
		 label = 1;
		 for row in spamreader:
		    info = row[0].split(',')
		    landmark.append([str(label), info[0],info[1]])
		    label += 1
                     
         with open(GTFile, 'rb') as csvfile:
		 spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
		 label = 1;
		 for row in spamreader:
		    info = row[0].split(',')
		    landmark_GT.append([str(label), info[0],info[1]])       
		    label += 1 
     
     elif solutionFile.endswith('.txt') and GTFile.endswith('.txt'):    
         f = open(solutionFile, 'r')
         label = 1;
         for line in f:
            info = line.split(' ')
            landmark.append([str(label), info[0],info[1]])
            label += 1 
            
         f = open(GTFile, 'r')
         label = 1;
         for line in f:
            info = line.split(' ')
            landmark_GT.append([str(label), info[0],info[1]])
            label += 1 
     
     landmarkError = []
     landmark = sorted(landmark)
     for i in range (0,len(landmark)-1):
         RelLandmarks = RelativeLandmarkPositions(landmark[i],landmark[i+1])
         RelLandmarksGT = RelativeLandmarkPositions(landmark_GT[i],landmark_GT[i+1])
         landmarkError.append(np.array(RelLandmarksGT) - np.array(RelLandmarks))
         
     errorLandmark = (1.0/(len(landmark)-1))*np.linalg.norm(landmarkError)
     
     print 'Your Solution Error', errorLandmark
     
     return errorLandmark
        
if __name__ == "__main__":
    
    '''
    For self testing only
    '''    
    solutionFile = '/home/turtlebot-admin/catkin_ws/src/rvss_workshop/scripts/map.txt'
    GTFile = '/home/turtlebot-admin/catkin_ws/src/rvss_workshop/scripts/GT.txt'
    error = ErrorFunction(solutionFile, GTFile)
    
