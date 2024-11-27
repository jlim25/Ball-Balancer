'''
Description: This file contains the controls and homing logic


'''
from class_camera import camera
import class_motorPosition as mc
import class_pid as pid

import time
import math
import numpy as np
import threading

import cv2

CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080

# For measuring performance of program 
#import time
# Track the start time
start_time = time.time()

# Loop counter
loop_count = 0

# image = np.zeros((640, 480, 3), dtype =np.uint8)
image = np.zeros((1080, 1920, 3), dtype =np.uint8)

ball_pos = [0, 0]
detectedBall = []

frame_count = 0
start_time = time.time()
img_start_time = time.time()
rob_start_time = time.time()
fps = 0.
img_fps = 0
rob_fps = 0

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
        
        self.pid_motorA = pid.PID()     # use default values P = 0.3, I = 0, D = 0.1
        self.pid_motorB = pid.PID()     # use default values P = 0.3, I = 0, D = 0.1
        self.pid_motorC = pid.PID()     # use default values P = 0.3, I = 0, D = 0.1

        self.center_x = CAMERA_WIDTH/2
        self.center_y = CAMERA_HEIGHT/2
        
        self.motor_vectors = {}  # Will hold motor unit vectors initialized dynamically
            
    def moveAllMotorsPos(self, position, speed):
        self.motorA.setMotorPosition(position, speed)       # Note: -ve degrees = counter clockwise
        self.motorB.setMotorPosition(position, speed)
        self.motorC.setMotorPosition(position, speed)
        
    def moveAllMotorsSpeed(self, speed):
        self.motorA.setMotorSpeed(speed)       # Note: -ve degrees = counter clockwise
        self.motorB.setMotorSpeed(speed)
        self.motorC.setMotorSpeed(speed)
    
    def offAllMotors(self):
        self.motorA.setMotorSpeed(0)
        self.motorB.setMotorSpeed(0)
        self.motorC.setMotorSpeed(0)

    def homing(self):
        """Homing routine using ArUco marker for position and pitch."""
        print("Starting homing routine...")
        homing_complete = False
        # Timeout threshold in seconds
        timeout_threshold = 5
        last_detection_time = time.time()  # Store the time of the last successful ArUco detection
        print(f"Timeout threshold: {timeout_threshold}s")

        while not homing_complete:
            # Capture frame from camera
            frame = self.cam.capture_image()

            # Detect the ArUco marker's pose (id, pitch, roll, distance, and position)
            marker_details = self.cam.detect_aruco(frame)
            
            if marker_details and marker_details[0] is not None:
                for marker in marker_details:
                    marker_id = marker['id']
                    pitch = marker['pitch']
                    roll = marker['roll']
                    distance = marker['distance']

                    if marker_id == 69: # and pitch is not None and distance is not None and roll is not None:
                        # Reset the last detection time whenever ArUco is detected
                        last_detection_time = time.time()
                        print(f"Current pitch: {pitch} degrees, Distance: {distance} meters")

                        # Calculate error in distance (Z-axis)
                        distance_error = distance - self.home_distance
                        pitch_error = pitch - self.home_pitch
                        roll_error = abs(roll) - self.home_roll

                        # Move robot based on distance error
                        if abs(distance_error) > 0.002:  # Tolerance of 0.2cm
                            if distance_error < 0:
                                # Turn all motors counterclockwise (increasing the height of the platform)
                                #self.moveAllMotorsPos(position=-10, speed=0.5)
                                self.moveAllMotorsSpeed(-0.3)       # -ve is counter-clockwise
                            else:
                                # Turn all motors clockwise (lowering the platform)
                                #self.moveAllMotorsPos(position=10, speed=0.5)
                                self.moveAllMotorsSpeed(0.3)        # +ve is clockwise
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
                            homing_complete = True
                            self.offAllMotors()
                            break
                # self.offAllMotors()
                # break
            else:
                if time.time() - last_detection_time > timeout_threshold:
                    print("Error: ArUco marker not detected for 5 seconds. Homing failed.")
                    return None  # Exit the function if the timeout is reached                

            # Slow down the loop to avoid overloading
            time.sleep(0.005)
        #12 V
        self.motorB.setMotorPosition(7, 0.5)
        self.motorA.setMotorPosition(3, 0.5)
        self.motorC.setMotorPosition(-10, 0.5)
        # 24V
        # self.motorB.setMotorPosition(6, 1.0)
        # self.motorA.setMotorPosition(3, 1.0)
        # self.motorC.setMotorPosition(-3, 1.0)
        
        self.offAllMotors()

        input("Press enter once homing is done and the aruco marker is removed...")
        
    def get_image_task(self):                
        global image, img_fps, img_start_time
        img_frame_count = 0
        print("Image capture thread started.")  # Debugging line
        while True:
            image[:] = self.cam.capture_image()
            # cv2.imshow('resized', image)
            img_frame_count += 1
            if img_frame_count == 100:
                img_end_time = time.time()
                img_elapsed_time = img_end_time - img_start_time
                img_fps = 100 / img_elapsed_time
                img_start_time = img_end_time
                img_frame_count = 0


    def detect_ball_task(self):
        global ball_pos, detectedBall, rob_fps, rob_start_time
        rob_frame_count = 0
        while True:
            detectedBall = self.cam.detect_ball(image)  # Ball detection based on the current image
            # print(f"detectedBall: {detectedBall}")
            if detectedBall:
                ball_pos = detectedBall[0]['position']
            rob_frame_count += 1
            if rob_frame_count == 100:
                rob_end_time = time.time()
                rob_elapsed_time = rob_end_time - rob_start_time
                rob_fps = 100 / rob_elapsed_time
                rob_start_time = rob_end_time
                rob_frame_count = 0
                
    ''' Helpfer functions for balancing '''
    def initialize_motor_vectors(self):
        """Initialize motor vectors using ArUco markers."""
        print("Attempting to detect 3 ArUco markers...")

        while True:
            # Capture frame from camera (or use the provided frame)
            frame = self.cam.capture_image()

            # Detect the ArUco markers' poses (returns a list of marker details)
            marker_details = self.cam.detect_aruco(frame)
            # Check if the return value from detect_aruco is not empty and contains valid markers
            if len(marker_details) == 3:
                print("Detected 3 markers successfully!")
                break  # Exit the loop if 3 markers are detected
            else:
                print(f"Detected None. retrying...")

            # Optionally, you can add a small delay to avoid overloading the system
            time.sleep(0.25)

        print(marker_details)

        # Map motor names to marker IDs
        motor_to_marker_map = {
            "motorA": 3,
            "motorB": 2,
            "motorC": 1
        }

        platform_center = np.array([self.center_x, self.center_y])

        for motor, marker_id in motor_to_marker_map.items():
            # Find the marker associated with the current motor
            marker = next((m for m in marker_details if m["id"] == marker_id), None)
            if not marker:
                raise ValueError(f"Marker with ID {marker_id} not detected!")

            # Calculate the vector from the platform center to the marker center
            marker_center = np.array(marker["center"])
            vector = marker_center - platform_center

            # Normalize the vector to make it a unit vector
            magnitude = np.linalg.norm(vector)
            if magnitude == 0:
                raise ValueError(f"Marker with ID {marker_id} is at the platform center!")
            unit_vector = vector / magnitude

            # Store the unit vector
            self.motor_vectors[motor] = unit_vector

        print(f"Motor vectors initialized: {self.motor_vectors}")
     
    def compute_motor_projections(self, error_vector):
        """Project the error vector onto dynamically initialized motor vectors."""
        if not self.motor_vectors:
            raise ValueError("Motor vectors have not been initialized.")

        # Get motor vectors
        motorA_vector = self.motor_vectors["motorA"]
        motorB_vector = self.motor_vectors["motorB"]
        motorC_vector = self.motor_vectors["motorC"]

        # Compute projections via dot product
        projA = np.dot(error_vector, motorA_vector)
        projB = np.dot(error_vector, motorB_vector)
        projC = np.dot(error_vector, motorC_vector)

        return projA, projB, projC

    def balance_ball(self, ball_position):
        """Balance the ball using PID control."""
        # Assume ball_position is a 2D vector [x, y] in platform coordinates
        target_position = np.array([self.center_x, self.center_y])  # Center of the platform
        error_vector = ball_position - target_position

        # Get projections onto motor vectors
        projA, projB, projC = self.compute_motor_projections(error_vector)

        # Compute PID outputs for each motor
        motorA_output = self.pid_motorA.computeOutput(currentPosition=projA, goal=0) 
        motorB_output = self.pid_motorB.computeOutput(currentPosition=projB, goal=0)
        motorC_output = self.pid_motorC.computeOutput(currentPosition=projC, goal=0)

        # TODO: tune PID

        # Apply motor outputs
        self.motorA.setMotorSpeed(motorA_output)
        self.motorB.setMotorSpeed(motorB_output)
        self.motorC.setMotorSpeed(motorC_output)

    def stop(self):
        """Stop all motors."""
        self.offAllMotors()
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
    # robot.stop()
    robot.initialize_motor_vectors()    # Should be called after homing

    input("Place the ball and press enter")

    try:
        imageCaptureThreadHandle = threading.Thread(target = robot.get_image_task)
        imageCaptureThreadHandle.start()

        processImageThreadHandle = threading.Thread(target = robot.detect_ball_task)
        processImageThreadHandle.start()
        time.sleep(0.05)
        
        while True:
            if detectedBall:
                print(f'ball_position: {ball_pos}')
                # robot.balance_ball(ball_pos)
            else:
                robot.offAllMotors()
                # break
            print(f"img_fps: {img_fps}, rob_fps: {rob_fps}")
            time.sleep(0.005)
            # if detectedBall:
            #     print(f'ball_position: {ball_pos}')
            #     robot.balance_ball(ball_pos)
            # else:
            #     robot.offAllMotors()
            #     break

        # imageCaptureThreadHandle.join()
        # processImageThreadHandle.join()

    finally:
        robot.stop()

'''
while loop 

 # # # Debug output for detected balls
            # for ball in detected_ball: # ball has x,y position of the ball, radius and area 
            #     # Get ball position, radius, and area
            #     position = ball['position']
            #     radius = ball['radius']
            #     area = ball['area']
                
            #     # Prepare the text to be displayed on the frame
            #     text = f"Pos: {position} R: {radius} A: {area}"

            #     # Use cv2.putText to draw the text on the frame
            #     cv2.putText(ball_frame, text, 
            #                 (position[0], position[1] - 10),  # Position for the text (slightly above the ball position)
            #                 cv2.FONT_HERSHEY_SIMPLEX,  # Font type
            #                 1.5,  # Font scale
            #                 (0, 255, 0),  # Color of the text (green)
            #                 2)  # Thickness of the text
            # # Show the frame with detected balls
            # cv2.imshow('Ball Detection', ball_frame)
            
            # ## CHECKING LOOPS PER SECOND
            # loop_count += 1
            # # Get the current time and calculate the elapsed time
            # elapsed_time = time.time() - start_time
            
            # # After 1 second, print the loop count and reset
            # if elapsed_time >= 1:
            #     print(f"Loops per second: {loop_count}")
            #     loop_count = 0
            #     start_time = time.time()


 try:
        while True:
            ball_frame = robot.cam.capture_image()
            # cv2.imshow('Ball Detection', ball_frame)
            detected_ball = robot.cam.detect_ball()     # should only return 1 'ball'
            
            if detected_ball:
                ball_position = detected_ball[0]['position']
                print(f'ball_position: {ball_position}')
                robot.balance_ball(ball_position)
            else:
                robot.offAllMotors()
                break
                
            # Press 'q' to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # time.sleep(0.01)  # Small delay for loop stability
            
    except KeyboardInterrupt:
        robot.stop()


'''