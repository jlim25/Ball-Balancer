import cv2

# Use GStreamer pipeline for Raspberry Pi camera
pipeline = "libcamerasrc ! video/x-raw, width=1920, height=1080, framerate=60/1 ! videoconvert ! appsink"
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        # Process the frame
        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
cap.release()
cv2.destroyAllWindows()