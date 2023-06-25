#############################################
# AUTHOR: Ruslan Masinjila
#############################################
import math
import random
import numpy as np
from Robot import *


###################################################################################################################################

robots         =  []
availableSteps =  []

###################################################################################################################################

def createRobots():
    numRobots        = 5
    numSteps         = 10
    stepSize         = 5
    
    for i in range(numRobots):
        robots.append(Robot(i,numSteps,stepSize))
        availableSteps.append(robots[-1].availableSteps)
        
###################################################################################################################################

createRobots()

    
    
while(True):
    canMove = [i for i in range(len(availableSteps)) if availableSteps[i] != 0]
    if canMove:
        movedRobot = random.choice(canMove)
        robots[movedRobot].moveRobot()
        availableSteps[movedRobot]=robots[movedRobot].availableSteps
    else:
        break
    print(availableSteps)
    
