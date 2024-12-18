import cv2
import cv2.aruco as aruco
import numpy as np
import math
from picamera2 import Picamera2
from libcamera import controls

from scipy.spatial.transform import Rotation as R
''' # For measuring performance of program 
#import time
# Track the start time
start_time = time.time()

# Loop counter
loop_count = 0
'''

class camera:
    def __init__(self, width=1920, height=1080, frame_rate=30, color_lower=None, color_upper=None, calibration_file='cameraCalibrate/calibration_chessboard.yaml'):
        # Initialize camera
        self.width = width
        self.height = height
        self.frame_rate = frame_rate
        self.color_lower = color_lower or np.array([10, 100, 100])  # Default orange lower bound
        self.color_upper = color_upper or np.array([150, 255, 255])  # Default orange upper bound
        self.middle = (int(self.width / 2), int(self.height / 2))
        self.cam = Picamera2()
        self._setup_camera()
    
        # Load the predefined dictionary for ArUco markers
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters_create()
        
        # Load calibration parameters
        self._load_calibration(calibration_file)
    
    def _setup_camera(self):
        """Configure and start the camera."""
        self.cam.configure(self.cam.create_video_configuration(main={"format": 'RGB888', "size": (self.width, self.height)}))
        self.cam.set_controls({"FrameRate": self.frame_rate})
        self.cam.start()
        self.cam.set_controls({"AfMode": controls.AfModeEnum.Continuous})
        
    def _load_calibration(self, calibration_file):
        """Load camera calibration parameters from a file."""
        cv_file = cv2.FileStorage(calibration_file, cv2.FILE_STORAGE_READ)
        self.matrix_coefficients = cv_file.getNode('K').mat()
        self.distortion = cv_file.getNode('D').mat()
        cv_file.release()
        
    def capture_image(self):    
        # Capture a frame from the camera
        frame = self.cam.capture_array()
        return frame
        
    def euler_from_quaternion(self, x, y, z, w):
        """Convert a quaternion into Euler angles (roll, pitch, yaw)."""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z  # in radians
        
    def detect_aruco_debug(self, frame): # this returns the frame and prints the pitch, roll, and distance onto the video feed
        """Detect ArUco markers and return their centers."""
        # Convert to grayscale for ArUco detection
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        # Detect markers
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        # print(f'id: {ids}')
        if ids is not None:
            for i, corner in enumerate(corners):
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    [corner], 0.011, self.matrix_coefficients, self.distortion)

                # Translation and rotation
                tvec = tvecs[0][0]
                rvec = rvecs[0][0]
                
                # Distance to marker
                distance = np.linalg.norm(tvec)
                
                # Convert rotation vector to Euler angles
                rotation_matrix = cv2.Rodrigues(rvec)[0]
                r = R.from_matrix(rotation_matrix)
                quat = r.as_quat()
                roll, pitch, yaw = self.euler_from_quaternion(quat[0], quat[1], quat[2], quat[3])
                roll, pitch, yaw = map(math.degrees, (roll, pitch, yaw))
                
                # Display pose information
                cv2.putText(frame, f"ID: {ids[i][0]}", (10, 30 + i * 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(frame, f"Dist: {distance:.2f}m", (10, 60 + i * 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                cv2.putText(frame, f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}", (10, 90 + i * 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                # Draw marker and axes
                aruco.drawDetectedMarkers(frame, corners)
                cv2.drawFrameAxes(frame, self.matrix_coefficients, self.distortion, rvec, tvec, 0.05)

        return frame

    def detect_aruco(self, frame): # this doesn't return a frame but the pitch, roll, and distance
        """Detect ArUco markers and return pitch, roll, and distance."""
        # Convert to grayscale for ArUco detection
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # Detect markers
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        # Initialize return variables
        pitch, roll, distance = None, None, None

        if ids is not None:
            for i, corner in enumerate(corners):
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    [corner], 0.011, self.matrix_coefficients, self.distortion)

                # Translation and rotation
                tvec = tvecs[0][0]  # Translation vector
                rvec = rvecs[0][0]  # Rotation vector

                # Distance to marker
                distance = np.linalg.norm(tvec)

                # Convert rotation vector to Euler angles
                rotation_matrix = cv2.Rodrigues(rvec)[0]
                r = R.from_matrix(rotation_matrix)  # Using scipy.spatial.transform.Rotation
                quat = r.as_quat()  # Quaternion representation
                roll, pitch, yaw = self.euler_from_quaternion(quat[0], quat[1], quat[2], quat[3])
                roll, pitch, yaw = map(math.degrees, (roll, pitch, yaw))  # Convert to degrees

                # Return the first detected marker's pose
                return pitch, roll, distance

        # If no markers detected, return None values
        return None, None, None

    def detect_ball(self, frame):
        """Capture a frame and detect the ball based on color and contour size."""        
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        # Mask the ball's color
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)

        # Apply morphological operations to clean up noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                
        # Find contours in the image
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        ball_data = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 20000:  # Adjust size threshold based on testing
                perimeter = cv2.arcLength(contour, True)
                circularity = 4 * np.pi * (area / (perimeter ** 2))
                if 0.4 < circularity <= 1.2:  # If the ping pong has text, it messes with this
                    # Draw the circle
                    # print(f"Circularity: {circularity}")
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    ball_data.append({"position": (int(x), int(y)), "radius": int(radius), "area": area})
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)  # Draw a green circle
        return ball_data # ball_data (dict) has position, radius, and area
    
    def release(self):
        """Release camera resources."""
        self.cam.stop()

# running class_camera.py directly 
if __name__ == "__main__":
    # Instantiate the BallDetector class
    cam = camera()

    try:
        while True:
            frame = cam.capture_image()
            '''
            # Detect ArUco markers
            frame = cam.detect_aruco_debug(frame)
            
            # Detect aruco markers and return (pitch, roll, distance)
            result = cam.detect_aruco(frame)
            print(result)
            print()
            '''
            detected_balls = cam.detect_ball(frame)

            # Debug output for detected balls
            for ball in detected_balls: # ball has x,y position of the ball, radius and area 
                # Get ball position, radius, and area
                position = ball['position']
                radius = ball['radius']
                area = ball['area']
                
                # Prepare the text to be displayed on the frame
                text = f"Pos: {position} R: {radius} A: {area}"

                # Use cv2.putText to draw the text on the frame
                cv2.putText(frame, text, 
                            (position[0], position[1] - 10),  # Position for the text (slightly above the ball position)
                            cv2.FONT_HERSHEY_SIMPLEX,  # Font type
                            1.5,  # Font scale
                            (0, 255, 0),  # Color of the text (green)
                            2)  # Thickness of the text
            # Show the frame with detected balls
            cv2.imshow('Ball Detection', frame)

            '''
            # Note: detect_aruco runs at 16-17 loops per second
            ## CHECKING LOOPS PER SECOND
            loop_count += 1
            # Get the current time and calculate the elapsed time
            elapsed_time = time.time() - start_time
            
            # After 1 second, print the loop count and reset
            if elapsed_time >= 1:
                print(f"Loops per second: {loop_count}")
                loop_count = 0
                start_time = time.time()
                
            '''

            # Press 'q' to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cam.release()
        cv2.destroyAllWindows()