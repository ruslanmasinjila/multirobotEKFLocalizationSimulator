#############################################
# AUTHOR: Ruslan Masinjila
#############################################
import math
import random
import numpy as np
from Robot import *

###################################################################################################################################
#########################################################################################################

def getRhoPhiMeasurements(observingRobot, targetRobot):
    
    # Simulate actual rho,phi measurements with zero-mean Gaussian noise
    xActual_targetRobot        = targetRobot.xActual[-1]
    yActual_targetRobot        = targetRobot.yActual[-1]
    
    xActual_observingRobot     = observingRobot.xActual[-1]
    yActual_observingRobot     = observingRobot.yActual[-1]
    thetaActual_observingRobot = observingRobot.thetaActual[-1]
    
    deltaRHO_observingRobot    = observingRobot.deltaRHO
    deltaPHI_observingRobot    = observingRobot.deltaPHI
    
    rho = (math.sqrt((xActual_targetRobot-xActual_observingRobot)**2  +        
                     (yActual_targetRobot-yActual_observingRobot)**2) +    
                      np.random.normal(0, deltaRHO_observingRobot))
    
    phi = (math.atan2(yActual_targetRobot-yActual_observingRobot, xActual_targetRobot-xActual_observingRobot) -             
                     thetaActual_observingRobot + np.random.normal(0, deltaPHI_observingRobot))     
    
    # Make sure the agle is between 0 and 2pi
    phi = (2*math.pi + phi)%(2*math.pi)
    
    observingRobot.rho = rho
    observingRobot.phi = phi
    
###################################################################################################################################

def runSimulation():

    robots         =  []
    availableMoves =  []

    numRobots        = 4
    numMoves         = 5
    moveSize         = 5
    
    for i in range(numRobots):
        robots.append(Robot(i,numMoves,moveSize))
        availableMoves.append(robots[-1].availableMoves)
        
    print(availableMoves) # toRemove
        
        
    while(True):

        # Determine which Robots have available moves left
        canMove = [i for i in range(len(availableMoves)) if availableMoves[i] != 0]
        
        if canMove:
        
            # Select a random robot to move
            robotToMove = random.choice(canMove)
            
            # Move the selected robot
            robots[robotToMove].moveRobot()
            
            # Rename for clarity.
            # Call the moved robot mR
            mR = robotToMove
            
            # Update the number of available moves of the moved robot
            availableMoves[mR]=robots[mR].availableMoves
            
            # A list of all stationary robots (i.e every other robot apart from mR)
            stationaryRobots = [i for i in range(len(availableMoves)) if i != mR]
            
            # Select one of the stationary robots for control input and pose estimation in the prediction step of the EKF
            # Call this robot pSR (Prediction Stationary Robot)
            pSR = random.choice(stationaryRobots)
            
            # The rest of the stationary robots are for pose correction in the correction step of the EKF
            # Call each of these robots cSR (Correction Stationary Robot)
            cSR = [i for i in stationaryRobots if i != pSR]
            

            
            print("Moved Robot:",mR) # toRemove
            print("Controller Stationary Robot:",pSR) # toRemove
            print("Corrector Stationary Robots:",cSR) # toRemove
            
            # This is where the movements, measurements, prediction and corrections happen
            
            # Get the actual robots using their IDs
            mR  = robots[mR]
            pSR = robots[pSR]
            
            # Get the relative Range-Bearing measurements between Moved Robot (mR) and the Prediction Stationary Robot (pSR)

            getRhoPhiMeasurements(mR, pSR)
            getRhoPhiMeasurements(pSR,mR)            
        else:
            break
            
runSimulation()

###################################################################################################################################


        
