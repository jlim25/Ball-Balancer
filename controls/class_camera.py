import cv2
import cv2.aruco as aruco
import numpy as np
from picamera2 import Picamera2
from libcamera import controls

class camera:
    def __init__(self, width=1920, height=1080, frame_rate=30, color_lower=None, color_upper=None):
        # Initialize camera
        self.width = width
        self.height = height
        self.frame_rate = frame_rate
        self.color_lower = color_lower or np.array([10, 100, 100])  # Default orange lower bound
        self.color_upper = color_upper or np.array([150, 255, 255])  # Default orange upper bound
        self.middle = (int(self.width / 2), int(self.height / 2))
        self.cam = Picamera2()
        #self.cam.set_controls({"AfMode": controls.AfModeEnum.Continuous})
        self._setup_camera()
    
        # Load the predefined dictionary for ArUco markers
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters_create()
    
    def _setup_camera(self):
        """Configure and start the camera."""
        self.cam.configure(self.cam.create_video_configuration(main={"format": 'RGB888', "size": (self.width, self.height)}))
        self.cam.set_controls({"FrameRate": self.frame_rate})
        self.cam.start()
        self.cam.set_controls({"AfMode": controls.AfModeEnum.Continuous})
        
    def capture_image(self):    
        # Capture a frame from the camera
        frame = self.cam.capture_array()
        return frame
        
    def detect_aruco(self, frame):
        """Detect ArUco markers and return their centers."""
        # Convert to grayscale for ArUco detection
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        # Detect markers
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        # print(f'id: {ids}')
        if ids is not None:
            for corner, marker_id in zip(corners, ids):
                # Get the center of the marker
                center_x = int((corner[0][0][0] + corner[0][2][0]) / 2)
                center_y = int((corner[0][0][1] + corner[0][2][1]) / 2)

                # Draw the marker and its center
                cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
                cv2.putText(frame, f"ID: {marker_id[0]}", (center_x, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                # Return the center and ID
                return (center_x, center_y), marker_id[0]
        
        return None, None

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
            # Detect ArUco markers
            marker_center, marker_id = cam.detect_aruco(frame)
            '''
            detected_balls = cam.detect_ball(frame)

            # Debug output for detected balls
            for ball in detected_balls:
                pass
                # print(f"Ball detected at {ball['position']} with radius {ball['radius']} and area {ball['area']}")
            '''
            # Show the frame with detected balls
            cv2.imshow('Ball Detection', frame)

            # Press 'q' to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cam.release()
        cv2.destroyAllWindows()