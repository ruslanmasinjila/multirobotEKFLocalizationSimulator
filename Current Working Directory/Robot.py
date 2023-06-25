import math
import random
import numpy as np
class Robot:
    

    def __init__(self, robotID,numMoves,moveSize):
    
        self.robotID                = robotID
        self.availableMoves         = numMoves
        self.moveSize               = moveSize
        
        # Actual Pose of the Robot (Ground Truth)
        self.xActual                = []
        self.yActual                = []
        self.thetaActual            = []
        
        # Estimated Pose of the robot (i.e the Pose Estimated by the EKF)
        self.xEstimated             = []
        self.yEstimated             = []
        self.thetaEstimated         = []
 
        self.xBar                      =   None
        self.yBar                      =   None
        self.thetaBar                  =   None
        self.sigmaBar                  =   None
        
        self.rho                       =   None
        self.rhoBar                    =   None
        self.deltaRHO                  =   0.0
        
        self.phi                       =   None
        self.phiBar                    =   None
        self.deltaPHI                  =   0.0
        
        self.moveRobot()

       

    def moveRobot(self):
        if(self.availableMoves>0):
            resolvingAngle = random.uniform(0, 2*math.pi) 
            if(len(self.xActual)==0):
                self.xActual.append(self.moveSize*math.cos(resolvingAngle))
                self.yActual.append(self.moveSize*math.sin(resolvingAngle))
                self.thetaActual.append(random.uniform(0, 2*math.pi)) 
                
                self.xEstimated.append(self.xActual[-1])
                self.yEstimated.append(self.yActual[-1])
                self.thetaEstimated.append(self.thetaActual[-1])
             
            else:
                self.xActual.append(self.xActual[-1]+self.moveSize*math.cos(resolvingAngle))
                self.yActual.append(self.yActual[-1]+self.moveSize*math.sin(resolvingAngle))
                self.thetaActual.append(random.uniform(0, 2*math.pi))
            
            self.availableMoves = self.availableMoves - 1