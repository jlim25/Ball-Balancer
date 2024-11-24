'''
Description: This file contains the controls and homing logic


'''
from class_camera import camera
import motorPosition as mc
import time

class robot:
    def __init__(self):
        # Initialize motors. These are classes
        self.motorA = mc.motorControl(mc.MOTOR_A_PWM_PIN, mc.MOTOR_A_DIR_PIN)
        self.motorB = mc.motorControl(mc.MOTOR_B_PWM_PIN, mc.MOTOR_B_DIR_PIN)
        self.motorC = mc.motorControl(mc.MOTOR_C_PWM_PIN, mc.MOTOR_C_DIR_PIN)
        
        # camera() is a class
        self.cam = camera()
        
        self.home_distance = 0.175       # meters. At rest, the distance is 0.15 m
        self.home_pitch = 0             # adjust as necessary (there is an offset error due to improper camera calibration)
        self.home_roll = 180            # rolls over at 180
    
    def moveAllMotors(self, position, speed):
        self.motorA.setMotorPosition(position, speed)       # Note: -ve degrees = counter clockwise
        self.motorB.setMotorPosition(position, speed)
        self.motorC.setMotorPosition(position, speed)
    
    def homing(self):
        """Homing routine using ArUco marker for position and pitch."""
        print("Starting homing routine...")

        # Timeout threshold in seconds
        timeout_threshold = 5
        last_detection_time = time.time()  # Store the time of the last successful ArUco detection
        print(f"Timeout threshold: {timeout_threshold}s")

        while True:
            # Capture frame from camera
            frame = self.cam.capture_image()

            # Detect the ArUco marker's pose (pitch, roll, and distance)
            pitch, roll, distance = self.cam.detect_aruco(frame)

            if pitch is not None and distance is not None and roll is not None:
                # Reset the last detection time whenever ArUco is detected
                last_detection_time = time.time()
                print(f"Current pitch: {pitch} degrees, Distance: {distance} meters")

                # Calculate error in distance (Z-axis)
                distance_error = distance - self.home_distance
                pitch_error = pitch - self.home_pitch
                roll_error = abs(roll) - self.home_roll

                # Move robot based on distance error
                if abs(distance_error) > 0.002:  # Tolerance of 1cm
                    if distance_error < 0:
                        # Turn all motors counterclockwise (increasing the height of the platform)
                        self.moveAllMotors(position=-10, speed=0.5)
                    else:
                        # Turn all motors clockwise (lowering the platform)
                        self.moveAllMotors(position=10, speed=0.5)
                '''
                # Adjust camera pitch if necessary
                if abs(pitch_error) > 1.5:  # Tolerance of 1 degree
                    if pitch_error > 0:
                        # Tilt camera up (reduce pitch)
                        self.motorC.setMotorPosition(5, 0.1)
                    else:
                        # Tilt camera down (increase pitch)
                        self.motorC.setMotorPosition(-5, 0.1)
                        
                # Adjust camera pitch if necessary
                if abs(roll_error) > 1.5:  # Tolerance of 1 degree
                    if roll_error > 0:
                        # Tilt camera up (reduce pitch)
                        self.motorC.setMotorPosition(5, 0.1)
                    else:
                        # Tilt camera down (increase pitch)
                        self.motorC.setMotorPosition(-5, 0.1)
                # Check if robot is close enough to the target position
                if abs(distance_error) < 0.01 and abs(pitch_error) < 1:
                    print("Homing complete!")
                    break
                '''
                if abs(distance_error) < 0.01:
                    print("Homing complete!")
                    break

            # Check for timeout: if 5 seconds have passed without detection, print an error
            if time.time() - last_detection_time > timeout_threshold:
                print("Error: ArUco marker not detected for 5 seconds. Homing failed.")
                break

            # Slow down the loop to avoid overloading
            time.sleep(0.005)

    def stop(self):
        """Stop all motors."""
        self.motorA.controller.off()
        self.motorB.controller.off()
        self.motorC.controller.off()
        self.cam.release()
        print("Robot stopped.")
    
    def control_posture(self, targetPosture):
        pitch = targetPosture[0]
        roll = targetPosture[1]
        pass
    
    
# To run the homing routine, instantiate the Robot class and call homing()
if __name__ == "__main__":
    robot = robot()
    robot.homing()
    robot.stop()