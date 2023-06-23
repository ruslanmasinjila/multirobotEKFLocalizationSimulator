import numpy as np
class Robot:
    
    def __init__(self, initialActualPose):
        
        # Pose  = [x,y,theta]
        # Sigma = 3x3 Matrix
        
        self.rho                       =   None
        self.phi                       =   None
        
        self.deltaRHO                  =   0.0
        self.deltaPHI                  =   0.0
        
        self.xBar                      =   None
        self.yBar                      =   None
        self.thetaBar                  =   None
        self.sigmaBar                  =   None
        
        self.actualPose                =   []
        self.estimatedPose             =   []
        self.covarianceMatrix          =   []
        
        self.actualPose.append(initialActualPose)
        self.estimatedPose.append(initialActualPose)
        self.covarianceMatrix.append(np.zeros((3,3)))
    
        
    def moveRobot(self, newActualPose):
        self.actualPose.append(newActualPose)