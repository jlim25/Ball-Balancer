import time

DEFAULT_KP = 1
DEFAULT_KD = 1 
DEFAULT_KI = 0

class PID:
    def __init__(self, pGain = DEFAULT_KP, dGain = DEFAULT_KD, iGain = DEFAULT_KI):
        self.kP = pGain
        self.kD = dGain
        self.kI = iGain
        self.previousError = 0
        self.previousTime = time.clock()
        self.integralBuildUp = 0
    
    def computeOutput(self, currentPosition, goal):
        currTime = time.clock()
        error = goal - currentPosition
        
        derivative = (currentPosition - self.previousError)/(currTime - self.previousTime)

        self.integralBuildUp += error*(currTime - self.previousTime)

        self.previousError = error
        self.previousTime = currTime

        return error*self.kP + derivative*self.kD + self.integralBuildUp*self.kI
    