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
    
    rho = (math.sqrt((xActual_observedRobot-xActual_observingRobot)**2 +        
                    (yActual_observedRobot-yActual_observingRobot)**2)+    
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
    
    phi_stationaryRobot            = stationaryRobot.phi
    phi_movedRobot                 = movedRobot.phi
    
    thetaPlusPhiStationaryRobot    = thetaEstimated_stationaryRobot + phi_stationaryRobot
    
    movedRobot.xBar                = (xEstimated_stationaryRobot + 
                                     rho_stationaryRobot*math.cos(thetaEstimated_stationaryRobot+phi_stationaryRobot))
    
    movedRobot.yBar                = (yEstimated_stationaryRobot + 
                                     rho_stationaryRobot*math.sin(thetaEstimated_stationaryRobot+phi_stationaryRobot))
    
    
    if(thetaPlusPhiStationaryRobot<=math.pi):
        movedRobot.thetaBar = (2*math.pi +(math.pi + thetaPlusPhiStationaryRobot  - phi_movedRobot))%(2*math.pi)
    else:
        movedRobot.thetaBar = (2*math.pi +(-math.pi + thetaPlusPhiStationaryRobot - phi_movedRobot))%(2*math.pi)
        
    
#################################################### 
# TESTING AREA
####################################################
'''
# Move Robot
# Take Relative Measurements
# Estimated Position
# Correct Position

# Create two test robots
movedRobot          = Robot(np.array([-10,2,4.1]))
stationaryRobot     = Robot(np.array([10,1,0]))

# Get relative Range Bearing Measurements
movedRobot          = getRangeBearingMeasurements(movedRobot, stationaryRobot)
stationaryRobot     = getRangeBearingMeasurements(stationaryRobot,movedRobot)

estimatePoseMovedRobot(movedRobot,stationaryRobot)
print([movedRobot.xBar,movedRobot.yBar,movedRobot.thetaBar])
print(list(movedRobot.actualPose[-1]))
print(list(movedRobot.estimatedPose[-1]))

movedRobot.moveRobot(np.array([5,5,1]))
movedRobot          = getRangeBearingMeasurements(movedRobot, stationaryRobot)
stationaryRobot     = getRangeBearingMeasurements(stationaryRobot,movedRobot)
estimatePoseMovedRobot(movedRobot,stationaryRobot)
print([movedRobot.xBar,movedRobot.yBar,movedRobot.thetaBar])
print(list(movedRobot.actualPose))
print(list(movedRobot.estimatedPose))

'''