import time

DEFAULT_KP = 0.3
DEFAULT_KI = 0
DEFAULT_KD = 0.1

class PID:
    def __init__(self, pGain = DEFAULT_KP, dGain = DEFAULT_KD, iGain = DEFAULT_KI):
        self.kP = pGain
        self.kD = dGain
        self.kI = iGain
        self.previousError = 0
        self.previousTime = time.perf_counter()
        self.integralBuildUp = 0
        self.integral_limit = 10.0  # Prevent integral windup (although we probably don't need it)
    
    def reset(self):
        """Reset the PID controller state."""
        self.previousError = 0.0
        self.previousTime = time.perf_counter()
        self.integralBuildUp = 0.0
    
    def computeOutput(self, currentPosition, goal):
        """Compute the PID output given the current position and goal."""
        currTime = time.perf_counter()
        dt = currTime - self.previousTime

        if dt <= 0:  # Prevent division by zero or negative time
            dt = 1e-6

        error = goal - currentPosition
        derivative = (error - self.previousError) / dt

        self.integralBuildUp += error * dt
        # Clamp the integral to prevent windup
        self.integralBuildUp = max(min(self.integralBuildUp, self.integral_limit), -self.integral_limit)

        # Update previous state
        self.previousError = error
        self.previousTime = currTime

        # Compute PID output
        return (error * self.kP) + (derivative * self.kD) + (self.integralBuildUp * self.kI)