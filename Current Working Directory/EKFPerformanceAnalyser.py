#############################################
# AUTHOR: Ruslan Masinjila
#############################################
import math
import random
import numpy as np
from multirobotEKFLocalizationSimulator import *


numSimulations  = 50
numRobots       = 5
numMoves        = 100
moveSize        = 3

NEES           = [[] for nR in range(numRobots)]
ANEES          = []


for nS in range(numSimulations):
    robots = runSimulation(numRobots,numMoves,moveSize)
    for r in robots:
        # Get the Actual Pose
        xActual      = r.xActual
        yActual      = r.yActual
        thetaActual  = r.thetaActual
        actualPose   = [np.array([[a],[b],[c]]) for a, b, c in zip(xActual, yActual, thetaActual)][1:]
        
        # Get the Estimated Pose
        xEstimated      = r.xEstimated
        yEstimated      = r.yEstimated
        thetaEstimated  = r.thetaEstimated
        estimatedPose   = [np.array([[a],[b],[c]]) for a, b, c in zip(xEstimated, yEstimated, thetaEstimated)][1:]
        
        # Get Sigma.
        sigma           = r.sigma[1:]
        
        anees           = [(np.transpose(a-b))@(np.linalg.inv(c))@(a-b) for a, b, c in zip(actualPose, estimatedPose, sigma)]
        anees           = [item[0][0] for item in anees]
        NEES[r.robotID].append(anees)
        
        
for i in NEES:
    average = list(map(sum, zip(*i)))
    average = [(1/(3*numSimulations))*x for x in average]
    ANEES.append(average)

print(ANEES[0])