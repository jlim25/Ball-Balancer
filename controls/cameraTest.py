import cv2
import numpy as np
from picamera2 import Picamera2, Preview
from libcamera import Transform
import time

# documentation: see https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf

# Initialize PiCamera
camera = Picamera2()
camera.video_configuration.controls.FrameRate = 60.0
camera.configure("video")
#camera.resolution = (640, 480)
camera.start_preview(Preview.QTGL, x=100, y=200, width=800, height=600,
					transform=Transform(hflip=1))

# Create a Array object to capture frames
#output = Picamera2.capture_array("main")

#print(camera.sensor_modes)


# Give the camera some time to warm up
camera.start() # Note it will only run for 2s without the sleep
time.sleep(15)
