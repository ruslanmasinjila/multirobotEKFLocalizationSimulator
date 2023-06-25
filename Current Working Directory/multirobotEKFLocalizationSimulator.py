#############################################
# AUTHOR: Ruslan Masinjila
#############################################
import math
import random
import numpy as np
from Robot import *


###################################################################################################################################

robots         =  []
availableMoves =  []

###################################################################################################################################

def createRobots():
    numRobots        = 4
    numMoves         = 5
    moveSize         = 5
    
    for i in range(numRobots):
        robots.append(Robot(i,numMoves,moveSize))
        availableMoves.append(robots[-1].availableMoves)
        
###################################################################################################################################

createRobots()
print(availableMoves)
    
    
while(True):
    canMove = [i for i in range(len(availableMoves)) if availableMoves[i] != 0]
    if canMove:
    
        # Select a random robot to move
        robotToMove = random.choice(canMove)
        
        # Move the selected robot
        robots[robotToMove].moveRobot()
        
        # Rename for clarity
        movedRobot = robotToMove
        
        # Update the number of Available moves
        availableMoves[movedRobot]=robots[movedRobot].availableMoves
        
        # All Stationary Robots
        stationaryRobots = [i for i in range(len(availableMoves)) if i != movedRobot]
        
        # Select one of the stationary robots for Control Input and pose estimation
        controllerStationaryRobot = random.choice(stationaryRobots)
        
        # The rest of stationary robots are for pose correction
        correctorStationaryRobot = [i for i in stationaryRobots if i != controllerStationaryRobot]
        

        
        print("Moved Robot:",movedRobot)
        print("Controller Stationary Robot:",controllerStationaryRobot)
        print("Corrector Stationary Robots:",correctorStationaryRobot)
    else:
        break
    
