import numpy as np
class Robot:
    
    def __init__(self, initialActualPose):
        
        # Pose  = [x,y,theta]
        # Sigma = 3x3 Matrix
        
        self.deltaRHO                  =   0.05
        self.deltaPHI                  =   0.1
        
        self.actualPose                =   []
        self.estimatedPose             =   []
        self.covarianceMatrix          =   []
        
        self.actualPose.append(initialActualPose)
        self.estimatedPose.append(initialActualPose)
        self.covarianceMatrix.append(np.zeros((3,3)))
    
        
    def addPose(self, newActualPose, newEstimatedPose, newCovarianceMatrix):
        self.actualPose.append(newActualPose)
        self.estimatedPose.append(newEstimatedPose)
        self.covarianceMatrix.append(newCovarianceMatrix)
        
    def getPose(self):
        return self.actualPose[-1],self.estimatedPose[-1],self.covarianceMatrix[-1]