import numpy as np
import math
from Robot import Robot


def getRangeBearingMeasurements(observingRobot, observedRobot):
    
    # Simulate rho,phi measurements with errors
    xActual_observedRobot      = observedRobot.actualPose[-1][0]
    yActual_observedRobot      = observedRobot.actualPose[-1][1]
    
    xActual_observingRobot     = observingRobot.actualPose[-1][0]
    yActual_observingRobot     = observingRobot.actualPose[-1][1]
    thetaActual_observingRobot = observingRobot.actualPose[-1][2]
    
    deltaRHO_observingRobot = observingRobot.deltaRHO
    deltaPHI_observingRobot = observingRobot.deltaPHI
    
    rho = (math.sqrt((xActual_observedRobot-xActual_observingRobot)**2  +        
                     (yActual_observedRobot-yActual_observingRobot)**2) +    
                      np.random.normal(0, deltaRHO_observingRobot))
    
    phi = (math.atan2(yActual_observedRobot-yActual_observingRobot, xActual_observedRobot-xActual_observingRobot) -             
                     thetaActual_observingRobot +
                     np.random.normal(0, deltaPHI_observingRobot))     
    
    phi = (2*math.pi + phi)%(2*math.pi)
    
    observingRobot.rho = rho
    observingRobot.phi = phi
       
    return observingRobot


# Estimate Pose of the moved Robot
def estimatePoseMovedRobot(movedRobot,stationaryRobot):
    
    
    xEstimated_stationaryRobot     = stationaryRobot.estimatedPose[-1][0]
    yEstimated_stationaryRobot     = stationaryRobot.estimatedPose[-1][1]
    thetaEstimated_stationaryRobot = stationaryRobot.estimatedPose[-1][2]
    
    rho_stationaryRobot            = stationaryRobot.rho
    deltaRHO_stationaryRobot       = stationaryRobot.deltaRHO
    
    phi_stationaryRobot            = stationaryRobot.phi
    deltaPHI_stationaryRobot       = stationaryRobot.deltaPHI
    
    rho_movedRobot                 = movedRobot.rho
    deltaRHO_movedRobot            = movedRobot.deltaRHO
    
    phi_movedRobot                 = movedRobot.phi
    deltaPHI_movedRobot            = movedRobot .deltaPHI
    
    # Estimate xBar, yBar and thetaBar of the moved Robot
    movedRobot.xBar                = (xEstimated_stationaryRobot + 
                                     rho_stationaryRobot*math.cos(thetaEstimated_stationaryRobot+phi_stationaryRobot))
    
    movedRobot.yBar                = (yEstimated_stationaryRobot + 
                                     rho_stationaryRobot*math.sin(thetaEstimated_stationaryRobot+phi_stationaryRobot))
    
    thetaPlusPhiStationaryRobot    = thetaEstimated_stationaryRobot + phi_stationaryRobot
    
    if(thetaPlusPhiStationaryRobot<=math.pi):
        movedRobot.thetaBar = (2*math.pi +(thetaPlusPhiStationaryRobot  - phi_movedRobot + math.pi))%(2*math.pi)
    else:
        movedRobot.thetaBar = (2*math.pi +(thetaPlusPhiStationaryRobot  - phi_movedRobot - math.pi))%(2*math.pi)
        
    # Estimate the covariance matrix of the moved Robot
    Ut  =  np.array([[(deltaRHO_stationaryRobot)**2,0,0],
                     [0,(deltaPHI_stationaryRobot)**2,0],
                     [0,0,(deltaPHI_movedRobot)**2]        
    ])
    
    Gmut = np.array([[1,0,-rho_stationaryRobot*math.sin(thetaPlusPhiStationaryRobot)],
                     [0,1, rho_stationaryRobot*math.cos(thetaPlusPhiStationaryRobot)],
                     [0,0,1]        
    ])
    
    Gut  = np.array([[math.cos(thetaPlusPhiStationaryRobot),-rho_stationaryRobot*math.sin(thetaPlusPhiStationaryRobot),0],
                     [math.sin(thetaPlusPhiStationaryRobot), rho_stationaryRobot*math.cos(thetaPlusPhiStationaryRobot),0],
                     [0,1,-1]        
    ])
    
    lastSigmaMovedRobot = movedRobot.sigma[-1]
    movedRobot.sigmaBar            = (Gmut)@(lastSigmaMovedRobot)@(np.transpose(Gmut)) + (Gut)@(Ut)@(np.transpose(Gut))
        
    
#################################################### 
# TESTING AREA
####################################################

# Move Robot
# Take Relative Measurements of ut
# Estimate Position (xBar, yBar and thetaBar) and covariance matrix
# Take the second relative measurements 
# Estimate Location of second Landmark using estimated positions only

# Create two test robots
movedRobot          = Robot(np.array([-10,2,4.1]))
stationaryRobot     = Robot(np.array([10,1,0]))

# Get relative Range Bearing Measurements
movedRobot          = getRangeBearingMeasurements(movedRobot, stationaryRobot)
stationaryRobot     = getRangeBearingMeasurements(stationaryRobot,movedRobot)

estimatePoseMovedRobot(movedRobot,stationaryRobot)
print([movedRobot.xBar,movedRobot.yBar,movedRobot.thetaBar])
print(list(movedRobot.actualPose))
print(list(movedRobot.estimatedPose))
print(list(movedRobot.sigmaBar))


