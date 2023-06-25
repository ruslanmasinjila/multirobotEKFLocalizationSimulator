#############################################
# AUTHOR: Ruslan Masinjila
#############################################
import math
import random
import numpy as np
from Robot import *

###################################################################################################################################


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
    
    observingRobot.Z = np.array([
                                 [observingRobot.rho],
                                 [observingRobot.phi]
    ])
    
###################################################################################################################################
# Estimate Pose of the Moving Robot (m) with the help of the Prediction Stationary Robot (p)
def estimatePoseMovingRobot(m,p):
    
    
    xEstimated_p     = p.xEstimated[-1]
    yEstimated_p     = p.yEstimated[-1]
    thetaEstimated_p = p.thetaEstimated[-1]
    
    rho_pm           = p.rho
    deltaRHO_pm      = p.deltaRHO
    
    phi_pm           = p.phi
    deltaPHI_pm      = p.deltaPHI
    
    phi_mp           = m.phi
    deltaPHI_mp      = m.deltaPHI
    
    # Estimate xBar, yBar and thetaBar of the moving Robot
    m.xBar            = (xEstimated_p + rho_pm*math.cos(thetaEstimated_p+phi_pm))
    
    m.yBar            = (yEstimated_p + rho_pm*math.sin(thetaEstimated_p+phi_pm))
    
    # Find the sum of Estimated Theta and Phi of robot p
    # Estimated Theta is the estimated angle robot p makes with the global frame of reference
    # phi_pm is the bearing of robot m w.r.t robot p
    thetaPlusPhi_p    = thetaEstimated_p + phi_pm
    
    if(thetaPlusPhi_p<=math.pi):
        m.thetaBar = (2*math.pi +(thetaPlusPhi_p  - phi_mp + math.pi))%(2*math.pi)
    else:
        m.thetaBar = (2*math.pi +(thetaPlusPhi_p  - phi_mp - math.pi))%(2*math.pi)
        
    # Calculate muBar
    m.muBar = np.array([
                        [m.xBar],
                        [m.yBar],
                        [m.thetaBar]
    ]
    )
        
    # Estimate the covariance matrix (SigmaBar) of the Moving Robot (m)
    Ut  =  np.array([[(deltaRHO_pm)**2,0,0],
                     [0,(deltaPHI_pm)**2,0],
                     [0,0,(deltaPHI_mp)**2]        
    ])
    
    Gmut = np.array([[1,0,-rho_pm*math.sin(thetaPlusPhi_p)],
                     [0,1, rho_pm*math.cos(thetaPlusPhi_p)],
                     [0,0,1]        
    ])
    
    Gut  = np.array([[math.cos(thetaPlusPhi_p),-rho_pm*math.sin(thetaPlusPhi_p),0],
                     [math.sin(thetaPlusPhi_p), rho_pm*math.cos(thetaPlusPhi_p),0],
                     [0,1,-1]        
    ])
    
    previousSigma_m = m.sigma[-1]
    
    # Calculate sigmaBar
    m.sigmaBar            = (Gmut)@(previousSigma_m)@(np.transpose(Gmut)) + (Gut)@(Ut)@(np.transpose(Gut))
    
###################################################################################################################################
def getRhoBarPhiBarMeasurements(m, c):
    
    xEstimated_c     = c.xEstimated[-1]
    yEstimated_c     = c.yEstimated[-1]
    
    xBar_m     = m.xBar
    yBar_m     = m.yBar
    thetaBar_m = m.thetaBar
    

    
    rhoBar = (math.sqrt((xEstimated_c-xBar_m)**2  +  (yEstimated_c-yBar_m)**2))
    
    phiBar = (math.atan2(yEstimated_c-yBar_m, xEstimated_c-xBar_m) - thetaBar_m)     
    
    phiBar = (2*math.pi + phiBar)%(2*math.pi)
    
    m.rhoBar = rhoBar
    m.phiBar = phiBar
    
     
    m.ZBar = np.array([
                      [m.rhoBar],
                      [m.phiBar]
    ])
           

###################################################################################################################################
# Update Pose of the Moving Robot (m)
def updatePoseMovingRobot(m,c):

    xEstimated_c      = c.xEstimated[-1]
    yEstimated_c      = c.yEstimated[-1]
    
    xBar                            = m.xBar
    yBar                            = m.yBar
    thetaBar                        = m.thetaBar
    
    ZBar                            = m.ZBar
    Z                               = m.Z
    
    sigma_c                         = c.sigma[-1]
    
    q  = (math.sqrt((xEstimated_c-xBar)**2  + (yEstimated_c-yBar)**2))
    
    Hr = (1/q)*np.array([[-(xEstimated_c-xBar),  -(yEstimated_c-yBar)  , 0],
                         [ (yEstimated_c-yBar)/q,-(xEstimated_c-xBar)/q,-q]
    ])
    
    Hl = (1/q)*np.array([[ (xEstimated_c-xBar),    (yEstimated_c-yBar),0],
                         [-(yEstimated_c-yBar)/q,(xEstimated_c-xBar)/q,0]
    ])
    
    Qt = np.array([[(m.deltaRHO)**2,0],
                   [0,(m.deltaPHI)**2]
    ])
    
    S = (Hr)@(m.sigmaBar)@(np.transpose(Hr)) + (Hl)@(sigma_c)@(np.transpose(Hl)) + Qt

    Kt   = (m.sigmaBar)@(np.transpose(Hr))@(np.linalg.inv(S))
    
    m.muBar = m.muBar + Kt@(Z-ZBar)
    m.sigmaBar  = (np.identity(3)-(Kt)@(Hr))@(m.sigmaBar)


def runSimulation():

    robots         =  []
    availableMoves =  []

    numRobots       = 3
    numMoves        = 5
    moveSize        = 5
    
    for i in range(numRobots):
        robots.append(Robot(i,numMoves,moveSize))
        availableMoves.append(robots[-1].availableMoves)
        
    while(True):

        # Determine which Robots have available moves left
        canMove = [i for i in range(len(availableMoves)) if availableMoves[i] != 0]
        
        if canMove:
        
            # Select a random robot to move
            robotToMove = random.choice(canMove)
            
            # Rename for clarity.
            # Call the moving robot m
            m = robotToMove
           
            # A list of all stationary robots (i.e every other robot apart from m)
            ps = [i for i in range(len(availableMoves)) if i != m]
            
            # Select one of the stationary robots for control input and pose estimation in the prediction step of the EKF
            # Call this robot p (Prediction Stationary Robot)
            p = random.choice(ps)
            
            # The rest of the stationary robots are for pose correction in the correction step of the EKF
            # Call each of these robots c (Correction Stationary Robot)
            C = [i for i in ps if i != p]
            

            ###################################################################################################
            # This is where the movements, measurements, prediction and corrections happen
            
            # Get the actual robots using their IDs
            m  = robots[m]
            p = robots[p]
            
            # Move the robot and update remaining available moves of the robot
            m.moveRobot()
            availableMoves[m.robotID]=robots[m.robotID].availableMoves
            
            # Get the relative Range-Bearing measurements between Moving Robot (m) and the Prediction Stationary Robot (p)
            getRhoPhiMeasurements(m, p)
            getRhoPhiMeasurements(p,m) 

            # Estimate the Pose and Covariance Matrix (Sigma) of the Moving Robot (m)  
            estimatePoseMovingRobot(m,p) 
            
 


            # Perform pose corrections using the remaining stationary robots (Correction Station Robots)
            
            for c in C:
            
                # Get the actual robot using its IDs
                c = robots[c]
                
                # Get Z of the Moving Robot (m)
                getRhoPhiMeasurements(m,c)
            
                # Estimate ZBar of the Moving Robot (m)
                getRhoBarPhiBarMeasurements(m, c)
                
                # Update the pose of the moving robot (m)
                updatePoseMovingRobot(m,c)
            m.xEstimated.append(m.muBar[0][0])
            m.yEstimated.append(m.muBar[1][0])
            m.thetaEstimated.append(m.muBar[2][0])
            m.sigma.append(m.sigmaBar)

        else:
            break
            
    return robots
            
robots = runSimulation()
###################################################################################################################################


        
