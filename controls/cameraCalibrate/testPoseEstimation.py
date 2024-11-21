import cv2 # Import the OpenCV library
import numpy as np # Import Numpy library
from scipy.spatial.transform import Rotation as R
import math # Math library

from picamera2 import Picamera2
from libcamera import controls
 
cam = Picamera2()
# we could try reducing resolution for better fps but it will reduce FOV
# FOV maxes out at 1920x1080p
height = 1080
width = 1920
middle = (int(width / 2), int(height / 2))
cam.configure(cam.create_video_configuration(main={"format": 'RGB888', "size": (width, height)}))
cam.set_controls({"FrameRate": 60})
cam.start()
cam.set_controls({"AfMode": controls.AfModeEnum.Continuous})
 
# Side length of the ArUco marker in meters 
aruco_marker_side_length = 0.011 # 0.005070 # side length of each cube. # Entire marker is 0.04154 m
# ^ This value was manually adjusted until the right distance was obtained. Not actual side length
 
# Calibration parameters yaml file
camera_calibration_parameters_filename = 'calibration_chessboard.yaml'
 
def euler_from_quaternion(x, y, z, w):
  """
  Convert a quaternion into euler angles (roll, pitch, yaw)
  roll is rotation around x in radians (counterclockwise)
  pitch is rotation around y in radians (counterclockwise)
  yaw is rotation around z in radians (counterclockwise)
  """
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
      
  return roll_x, pitch_y, yaw_z # in radians
 
def main():
  """
  Main method of the program.
  """
 
  # Load the camera parameters from the saved file
  cv_file = cv2.FileStorage(
    camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ) 
  mtx = cv_file.getNode('K').mat()
  dst = cv_file.getNode('D').mat()
  cv_file.release()
     
  # Load the ArUco dictionary
  this_aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
  this_aruco_parameters = cv2.aruco.DetectorParameters_create()
   
  while(True):
  
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    frame = cam.capture_array() 
     
    # Detect ArUco markers in the video frame
    (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(
      frame, this_aruco_dictionary, parameters=this_aruco_parameters)
       
    # Check that at least one ArUco marker was detected
    if marker_ids is not None:
 
      # Draw a square around detected markers in the video frame
      cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids)
       
      # Get the rotation and translation vectors
      rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
        corners,
        aruco_marker_side_length,
        mtx,
        dst)
         
      # Print the pose for the ArUco marker
      # The pose of the marker is with respect to the camera lens frame.
      # Imagine you are looking through the camera viewfinder, 
      # the camera lens frame's:
      # x-axis points to the right
      # y-axis points straight down towards your toes
      # z-axis points straight ahead away from your eye, out of the camera
      for i, marker_id in enumerate(marker_ids):
       
        # Store the translation (i.e. position) information
        transform_translation_x = tvecs[i][0][0]
        transform_translation_y = tvecs[i][0][1]
        transform_translation_z = tvecs[i][0][2]
 
        # Calculate distance to marker
        distance = math.sqrt(transform_translation_x**2 + transform_translation_y**2 + transform_translation_z**2)
 
        # Store the rotation information
        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
        r = R.from_matrix(rotation_matrix[0:3, 0:3])
        quat = r.as_quat()   
         
        # Quaternion format     
        transform_rotation_x = quat[0] 
        transform_rotation_y = quat[1] 
        transform_rotation_z = quat[2] 
        transform_rotation_w = quat[3] 
         
        # Euler angle format in radians
        roll_x, pitch_y, yaw_z = euler_from_quaternion(transform_rotation_x, 
                                                       transform_rotation_y, 
                                                       transform_rotation_z, 
                                                       transform_rotation_w)
         
        roll_x = math.degrees(roll_x)
        pitch_y = math.degrees(pitch_y)
        yaw_z = math.degrees(yaw_z)
        # print("transform_translation_x: {}".format(transform_translation_x))
        # print("transform_translation_y: {}".format(transform_translation_y))
        # print("transform_translation_z: {}".format(transform_translation_z))
        # print("roll_x: {}".format(roll_x))
        # print("pitch_y: {}".format(pitch_y))
        # print("yaw_z: {}".format(yaw_z))
        # print()
         
        # Draw the axes on the marker
        # cv2.aruco.drawAxis(frame, mtx, dst, rvecs[i], tvecs[i], 0.05) #outdated
        cv2.drawFrameAxes(frame, mtx, dst, rvecs[i], tvecs[i], 0.05)
        
        # Display roll, pitch, yaw, and distance on the frame
        cv2.putText(frame, f"Roll: {roll_x:.2f}", (10, 30 + 30 * i), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, f"Pitch: {pitch_y:.2f}", (10, 60 + 30 * i), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Yaw: {yaw_z:.2f}", (10, 90 + 30 * i), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.putText(frame, f"Distance: {distance:.2f} m", (10, 120 + 30 * i), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                
     
    # Display the resulting frame
    cv2.imshow('frame',frame)
          
    # If "q" is pressed on the keyboard, 
    # exit this loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
  
  # Close down the video stream
  cam.stop()
  cv2.destroyAllWindows()
   
if __name__ == '__main__':
  print(__doc__)
  main()