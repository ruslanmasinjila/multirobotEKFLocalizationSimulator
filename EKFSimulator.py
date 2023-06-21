import numpy as np
import math
from Robot import Robot


def getRangeBearingMeasurements(movingRobot, landmarkRobot):
    
    # Simulate rho,phi measurements with errors
    # Assume the moving robot is taking the measurements
    rho = math.sqrt((landmarkRobot.actualPose[-1][0]-movingRobot.actualPose[-1][0])**2 +           \
                    (landmarkRobot.actualPose[-1][1]-movingRobot.actualPose[-1][1])**2)+           \
                     np.random.normal(0, movingRobot.deltaRHO)
    
    phi = math.atan2(landmarkRobot.actualPose[-1][1]-movingRobot.actualPose[-1][1],                \
                     landmarkRobot.actualPose[-1][0]-movingRobot.actualPose[-1][0]) -              \
                     movingRobot.actualPose[-1][2] + np.random.normal(0, movingRobot.deltaPHI)     
    
    phi = (2*math.pi + phi)%(2*math.pi)
          
    
    print(rho,phi)

movingRobot    = Robot(np.array([1,-1,0]))
landmarkRobot  = Robot(np.array([0,0,0]))

getRangeBearingMeasurements(movingRobot, landmarkRobot)