import class_camera
import class_robot
import cv2

camera = class_camera.BallDetector()    # use default values
robot = class_robot()

try:
    while True:
        frame, detected_balls = camera.detect_ball()

        # Debug output for detected balls
        for ball in detected_balls:
            print(f"Ball detected at {ball['position']} with radius {ball['radius']} and area {ball['area']}")

        # Show the frame with detected balls
        cv2.imshow('Ball Detection', frame)

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    camera.release()
    cv2.destroyAllWindows()