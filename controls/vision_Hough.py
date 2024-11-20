import cv2
import numpy as np
from picamera2 import Picamera2

# Initialize the camera
cam = Picamera2()
height = 1080
width = 1920

# Configure the camera to capture video
cam.configure(cam.create_video_configuration(main={"format": 'RGB888', "size": (width, height)}))
cam.set_controls({"FrameRate": 60})
cam.start()

# Define HSV color range for the ping pong ball (adjust if needed)
ball_color_lower = np.array([10, 100, 100])  # Orange lower bound
ball_color_upper = np.array([150, 255, 255])  # Orange upper bound

while True:
    # Capture a frame from the camera
    frame = cam.capture_array()

    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    # Create a mask for the ball color
    mask = cv2.inRange(hsv, ball_color_lower, ball_color_upper)
    #cv2.imshow('Ping Pong Ball Detection', mask)	

    # Apply GaussianBlur to reduce noise
    blurred = cv2.GaussianBlur(mask, (5, 5), 0)
    #cv2.imshow('Ping Pong Ball Detection', blurred)

    # Detect circles using HoughCircles
    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, dp=2.0, minDist=1024, \
    param1=50, param2=30, minRadius=50, maxRadius=500)

    # If circles are detected, draw them
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for circle in circles[0, :]:
            center = (circle[0], circle[1])  # Circle center
            radius = circle[2]  # Circle radius
            # Draw the circle
            cv2.circle(frame, center, radius, (0, 255, 0), 4)  # Green circle
            # Draw the center
            cv2.circle(frame, center, 3, (0, 0, 255), -1)  # Red center dot

    # Show the frame with the detected ping pong ball
    cv2.imshow('Ping Pong Ball Detection', frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Properly shutdown the camera
cam.stop()
cv2.destroyAllWindows()
