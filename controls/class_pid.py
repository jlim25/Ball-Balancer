import time

# DEFAULT_KP = 0.0005
# DEFAULT_KP = 0.0005
# DEFAULT_KI = 0.000002
# DEFAULT_KD = 0.002

#best so far by daniel
# DEFAULT_KP = 0.12
# DEFAULT_KI = 0.0015
# DEFAULT_KD = 0.045

DEFAULT_KP = 0.01
DEFAULT_KI = 0.001
DEFAULT_KD = 0.15

# best thus far
# DEFAULT_KP = 0.05
# DEFAULT_KI = 0
# DEFAULT_KD = 0.05

class PID:
    def __init__(self, pGain = DEFAULT_KP, dGain = DEFAULT_KD, iGain = DEFAULT_KI):
        self.kP = pGain
        self.kD = dGain
        self.kI = iGain
        self.previousError = 0
        self.previousTime = time.perf_counter()
        self.integralBuildUp = 0
        self.integral_limit = 1.0  # Prevent integral windup (although we probably don't need it)
        self.integralValue = 0
        self.previousDerivative = 0
        self.previousValue = 0
        self.alpha = 1.0
    
    def reset(self):
        """Reset the PID controller state."""
        self.previousError = 0.0
        self.previousTime = time.perf_counter()
        self.integralBuildUp = 0.0
    
    # def computeOutput(self, currentPosition, goal):
    #     """Compute the PID output given the current position and goal."""
    #     currTime = time.perf_counter()
    #     # dt = currTime - self.previousTime
    #     dt = 0.0005

    #     if dt <= 0:  # Prevent division by zero or negative time
    #         dt = 1e-6

    #     error = goal - currentPosition
    #     print(f"error: {error}")
    #     # derivative = (error - self.previousError) / dt #old
    #     alpha = 0.9  # Smoothing factor (0 < alpha < 1)
    #     derivative = alpha * self.previousDerivative + (1 - alpha) * ((error - self.previousError) / dt)
    #     self.previousDerivative = derivative

    #     self.integralBuildUp += error * dt
    #     # Clamp the integral to prevent windup
    #     self.integralBuildUp = max(min(self.integralBuildUp, self.integral_limit), -self.integral_limit)

    #     # Update previous state
    #     self.previousError = error
    #     self.previousTime = currTime

    #     output = (error * self.kP) + (derivative * self.kD) + (self.integralBuildUp * self.kI)
        # if abs(output) < 0.01:
        #     return 0
    #     # Compute PID output
    #     return output
    
    def computeOutput(self, currentPosition, goal):
        current_time = time.perf_counter()
        if self.previousTime is None:
            self.previousTime = current_time
            return 0, 0
        # Computer Error
        error = goal - currentPosition
        # Integral
        self.integralValue += error * (current_time - self.previousTime)
        # Derivative
        derivative = (error - self.previousError) / (current_time - self.previousTime)
        # PID Output
        output = self.kP * error + self.kI * self.integralValue + self.kD * derivative
        # Smooth output
        output = self.alpha * output + (1 - self.alpha) * self.previousValue
        
        # convert output to speed
        result = output / 100      
        
        # print(result)
        if abs(result) < 0.01:
            result = 0  

        self.previousError = error
        self.previousValue = result
        self.previousTime = current_time

        return result