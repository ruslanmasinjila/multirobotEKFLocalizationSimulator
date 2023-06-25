#############################################
# AUTHOR: Ruslan Masinjila
#############################################
import math
import random
import numpy as np
from multirobotEKFLocalizationSimulator import *

numRobots       = 3
numMoves        = 5
moveSize        = 3

robots = runSimulation(numRobots,numMoves,moveSize)

ANEES = []

for r in robots:
    # Get the Actual Pose
    xActual      = r.xActual
    yActual      = r.yActual
    thetaActual  = r.thetaActual
    actualPose   = [np.array([[a],[b],[c]]) for a, b, c in zip(xActual, yActual, thetaActual)]
    
    # Get the Estimated Pose
    xEstimated      = r.xEstimated
    yEstimated      = r.yEstimated
    thetaEstimated  = r.thetaEstimated
    estimatedPose   = [np.array([[a],[b],[c]]) for a, b, c in zip(xEstimated, yEstimated, thetaEstimated)]
    
    # Get Sigma
    sigma           = r.sigma