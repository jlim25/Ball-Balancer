
import cv2
import numpy as np
from picamera2 import Picamera2

cam = Picamera2()
# we could try reducing resolution for better fps but it will reduce FOV
# FOV maxes out at 1920x1080p
height = 1080
width = 1920
middle = (int(width / 2), int(height / 2))
cam.configure(cam.create_video_configuration(main={"format": 'RGB888', "size": (width, height)}))
cam.set_controls({"FrameRate": 60})
#picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
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
    # Apply the mask to get the white regions of the image
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # Convert the result to grayscale to prepare for contour detection
    gray = cv2.cvtColor(result, cv2.COLOR_RGB2GRAY)

    # Apply a blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (15, 15), 0)
    #cv2.imshow('Ping Pong Ball Detection', blurred)

    # Find contours in the image
    contours, _ = cv2.findContours(blurred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Debug: Highlight specific contours based on size
    for contour in contours:
        area = cv2.contourArea(contour)
		# at max heigh: the area is rougly 22000
        if area > 20000:  # Filter out small noise (adjust the threshold as needed)
            # Draw bounding circles around filtered contours
            (x, y), radius = cv2.minEnclosingCircle(contour)
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)  # Green circle
            # debug
            print(f"Contour at ({x:.2f}, {y:.2f}) with radius {radius:.2f} and area {area:.2f}")

    # Show the original frame with drawn contours
    cv2.imshow('Contours', frame)
    
    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
