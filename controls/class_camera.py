import cv2
import numpy as np
from picamera2 import Picamera2

class BallDetector:
    def __init__(self, width=1920, height=1080, frame_rate=60, color_lower=None, color_upper=None):
        # Initialize camera
        self.width = width
        self.height = height
        self.frame_rate = frame_rate
        self.color_lower = color_lower or np.array([10, 100, 100])  # Default orange lower bound
        self.color_upper = color_upper or np.array([150, 255, 255])  # Default orange upper bound
        self.middle = (int(self.width / 2), int(self.height / 2))
        self.cam = Picamera2()
        self._setup_camera()
    
    def _setup_camera(self):
        """Configure and start the camera."""
        self.cam.configure(self.cam.create_video_configuration(main={"format": 'RGB888', "size": (self.width, self.height)}))
        self.cam.set_controls({"FrameRate": self.frame_rate})
        self.cam.start()

    def detect_ball(self):
        """Capture a frame and detect the ball based on color and contour size."""
        # Capture a frame from the camera
        frame = self.cam.capture_array()

        # Convert the frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        # Create a mask for the ball color
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)

        # Apply the mask to isolate the ball
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Convert the result to grayscale and blur
        gray = cv2.cvtColor(result, cv2.COLOR_RGB2GRAY)
        blurred = cv2.GaussianBlur(gray, (15, 15), 0)

        # Find contours in the image
        contours, _ = cv2.findContours(blurred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        ball_data = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 23000:  # Adjust size threshold based on testing
                (x, y), radius = cv2.minEnclosingCircle(contour)
                ball_data.append({"position": (int(x), int(y)), "radius": int(radius), "area": area})
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)  # Draw a green circle
        
        return frame, ball_data

    def release(self):
        """Release camera resources."""
        self.cam.stop()

if __name__ == "__main__":
    # Instantiate the BallDetector class
    detector = BallDetector()

    try:
        while True:
            frame, detected_balls = detector.detect_ball()

            # Debug output for detected balls
            for ball in detected_balls:
                print(f"Ball detected at {ball['position']} with radius {ball['radius']} and area {ball['area']}")

            # Show the frame with detected balls
            cv2.imshow('Ball Detection', frame)

            # Press 'q' to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        detector.release()
        cv2.destroyAllWindows()