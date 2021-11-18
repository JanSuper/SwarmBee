import cv2
import cv2.aruco as aruco
import os
import pickle
import numpy as np
import math
from lib.tello import Tello
import time

# Specify the marker id to find
marker_id = 3

# Construct a Tello instance so we can communicate with it over UDP
tello = Tello()

# Send the command string to wake Tello up
tello.send("command")
# tello.send("downvision 1")

# Delay
time.sleep(1)

# Initialize the video stream which will start sending to port 11111
tello.send("streamon")


# Get Tello stream
cap = cv2.VideoCapture('udp://127.0.0.1:11111')

# Set the camera size - must be consistent with size of calibration photos
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# Specify marker length and params
marker_length = 2 # was 175mm/17.5cm
aruco_params = aruco.DetectorParameters_create()
aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

# Font for displaying text on screen
font = cv2.FONT_HERSHEY_SIMPLEX

# Load camera calibration data
if not os.path.exists('./tello_calibration.pckl'):
    print("Missing Tello camera calibration file: tello_calibration.pckl")
    exit()
else:
    f = open('tello_calibration.pckl', 'rb')
    (camera_matrix, distortion_coefficients, _, _) = pickle.load(f)
    f.close()
    if camera_matrix is None or distortion_coefficients is None:
        print("Calibration issue. You may need to recalibrate.")
        exit()

# Keep a moving average of yaw
sample_count = 30
yaw_vals = []
min_yaw_range, max_yaw_range = (-10, 10)

while True:

    # Read the camera frame
    ret, frame = cap.read()

    # Convert to gray scale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Find all the aruco markers in the image
    corners, ids, _ = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=aruco_params, cameraMatrix=camera_matrix, distCoeff=distortion_coefficients)

    # Detect ID specified above
    if ids is not None and ids[0] == marker_id:

        # Draw the marker boundaries
        aruco.drawDetectedMarkers(frame, corners, ids, (0, 255, 0))
        
        # Get the marker pose
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, distortion_coefficients)

        # Unpack the rotation and translation values
        rvec = rvec[0, 0, :]
        tvec = tvec[0, 0, :]

        # Plot a point at the center of the image
        cv2.circle(frame, (480, 360), 2, (255, 255, 255), -1)

        # Draw x (red), y (green), z (blue) axes
        aruco.drawAxis(frame, camera_matrix, distortion_coefficients, rvec, tvec, marker_length)

        # Draw black background for text
        cv2.rectangle(frame, (0, 600), (800, 720), (0, 0, 0), -1)

        # Display the xyz position coordinates
        position = "Marker %d position: x=%4.0f y=%4.0f z=%4.0f"%(marker_id, tvec[0], tvec[1], tvec[2])
        cv2.putText(frame, position, (20, 650), font, 1, (255, 255, 255), 1, cv2.LINE_AA)

        # Create empty rotation matrix
        rotation_matrix = np.zeros(shape=(3,3))

        # Convert rotation vector to rotation matrix
        cv2.Rodrigues(rvec, rotation_matrix, jacobian = 0)

        # Get yaw/pitch/roll of rotation matrix
        # We are most concerned with rotation around pitch axis which translates to Tello's yaw
        ypr = cv2.RQDecomp3x3(rotation_matrix)

        # Display the yaw/pitch/roll angles
        attitude2 = "Marker %d attitude: y=%4.0f p=%4.0f r=%4.0f"%(marker_id, ypr[0][0], ypr[0][1], ypr[0][2])
        cv2.putText(frame, attitude2, (20, 700), font, 1, (255, 255, 255), 1, cv2.LINE_AA)

        yaw_vals.append(ypr[0][1])

        if len(yaw_vals) > sample_count:

            yaw_avg = int(sum(yaw_vals)/len(yaw_vals))

            print("Yaw average: ", yaw_avg)

            # If yaw average is negative then yaw left (ccw)
            if yaw_avg < min_yaw_range:
                tello.send("ccw " + str(abs(yaw_avg)))
                print("yawing left")
            # If yaw average is positive then yaw right (cw)
            elif yaw_avg > max_yaw_range:
                tello.send("cw " + str(abs(yaw_avg)))
                print("yawing right")

            # Reset the array
            yaw_vals = []

    cv2.imshow("Tello", frame)
    
    # use 'q' to quit
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        tello.close()
        break

    if key == ord(' '):
        file_path = os.getcwd()
        file_name = time.strftime("%Y%m%d-%H%M%S")
        cv2.imwrite(file_path + "/" + file_name + ".jpg", frame)
        # tello.send('takeoff')

