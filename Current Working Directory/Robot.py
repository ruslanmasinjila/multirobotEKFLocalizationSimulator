import math
import random
import numpy as np
class Robot:
    

    def __init__(self, robotID,numSteps,stepSize):
    
        self.robotID                = robotID
        self.availableSteps         = numSteps
        self.stepSize               = stepSize
        
        self.xActual                = []
        self.yActual                = []
        self.thetaActual            = []
        
        self.xEstimated             = []
        self.yEstimated             = []
        self.thetaEstimated         = []
        
        
        self.moveRobot()

       

    def moveRobot(self):
        if(self.availableSteps>0):
            resolvingAngle = random.uniform(0, 2*math.pi) 
            if(len(self.xActual)==0):
                self.xActual.append(self.stepSize*math.cos(resolvingAngle))
                self.yActual.append(self.stepSize*math.sin(resolvingAngle))
                self.thetaActual.append(random.uniform(0, 2*math.pi)) 
                
                self.xEstimated.append(self.xActual[-1])
                self.yEstimated.append(self.yActual[-1])
                self.thetaEstimated.append(self.thetaActual[-1])
             
            else:
                self.xActual.append(self.xActual[-1]+self.stepSize*math.cos(resolvingAngle))
                self.yActual.append(self.yActual[-1]+self.stepSize*math.sin(resolvingAngle))
                self.thetaActual.append(random.uniform(0, 2*math.pi))
            
            self.availableSteps = self.availableSteps - 1
        
        
        
'''       
        # Pose  = [x,y,theta]
        # Sigma = 3x3 Matrix
        
        self.rho                       =   None
        self.phi                       =   None
        self.rhoBar                    =   None
        self.phiBar                    =   None
        
        self.deltaRHO                  =   0.1
        self.deltaPHI                  =   0.05
        
        self.xBar                      =   None
        self.yBar                      =   None
        self.thetaBar                  =   None
        self.sigmaBar                  =   None
        self.mu                        =   None
        
        self.actualPose                =   []
        self.estimatedPose             =   []
        self.sigma                     =   []
        
        self.actualPose.append(initialActualPose)
        self.estimatedPose.append(initialActualPose)
        self.sigma.append(np.zeros((3,3)))
    
        
    def moveRobot(self, newActualPose):
        self.actualPose.append(newActualPose)
        
'''