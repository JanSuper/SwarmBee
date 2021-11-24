import cv2
import cv2.aruco as aruco
import os
import pickle
import numpy as np
import math
from lib.tello import Tello
import time

### Marker positions in room ###

#  0----4----1
#  -----------
#  7----8----5
#  -----------
#  3----6----2

### Marker 0 is the origin of coordinate system
# distance between corners EDGE (measured from one marker's left top corner to the other's left top)
# so distance between marker 0 and 1 is EDGE along x-axis; between marker 0 and 3 is EDGE along y-axis etc.
# and distance between marker 0 and 4 is EDGE/2 along x-axis; between marker 0 and 7 is EDGE/2 along y-axis etc.
# (actual drone coordinates are taken relative to the center of the marker)

EDGE = 200  # 200 cm is the distance between markers in the corners

# Marker IDs used:
markers = [0, 1, 2, 3, 4, 5, 6, 7, 8]

# x and y offset for each marker:
offsets = [[0, 0], [EDGE, 0], [EDGE, EDGE], [0, EDGE],
           [EDGE/2, 0], [EDGE, EDGE/2], [EDGE/2, EDGE], [0, EDGE/2], [EDGE/2, EDGE/2]]

# Construct a Tello instance so we can communicate with it over UDP
tello = Tello()

# Send the command string to wake Tello up
tello.send("command")
#tello.send("downvision 1")

# Delay
time.sleep(1)

# Initialize the video stream which will start sending to port 11111
tello.send("streamoff")
tello.send("streamon")

# Get Tello stream
cap = cv2.VideoCapture('udp://0.0.0.0:11111')  # 0.0.0.0:11111 or 127.0.0.1:11111

#cap = cv2.VideoCapture('./utils/20191219-104330.avi') # Tello video from stream

# Set the camera size - must be consistent with size of calibration photos
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# Specify marker length and params
# TODO need to get exact measurement after printing out on A4 paper (all markers MUST have same size)
marker_length = 18
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

while True:

    # Read the camera frame
    ret, frame = cap.read()

    img_aruco = frame

    # Convert to gray scale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Find all the aruco markers in the image
    corners, ids, _ = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=aruco_params, cameraMatrix=camera_matrix, distCoeff=distortion_coefficients)

    # Detect ID specified above
    if ids is not None:

        found_marker = ids[0][0]

        # Let's find one of the markers in the room (used marker ids: 0 ... 8)
        if found_marker in range(0, 9):

            marker_id = found_marker

            # Draw the marker boundaries
            img_aruco = aruco.drawDetectedMarkers(frame, corners, ids, (0, 255, 0))

            # Get the marker pose
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, distortion_coefficients)

            # Unpack the rotation and translation values
            rvec = rvec[0, 0, :]
            tvec = tvec[0, 0, :]

            # Plot a point at the center of the image
            cv2.circle(img_aruco, (480, 360), 2, (255, 255, 255), -1)

            # Draw x (red), y (green), z (blue) axes
            img_aruco = aruco.drawAxis(img_aruco, camera_matrix, distortion_coefficients, rvec, tvec, marker_length)

            # Draw black background for text
            cv2.rectangle(img_aruco, (0, 600), (800, 720), (0, 0, 0), -1)

            # Display the xyz position coordinates
            x = tvec[0] + offsets[marker_id][0]
            y = tvec[1] + offsets[marker_id][1]
            z = tvec[2]
            position = "Marker %d position: x=%4.0f y=%4.0f z=%4.0f"%(marker_id, x, y, z)
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

    else:
        img_aruco = frame

    cv2.imshow("Tello", img_aruco)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break

    if key == ord(' '):
        file_path = os.getcwd()
        file_name = time.strftime("%Y%m%d-%H%M%S")
        cv2.imwrite(file_path + "/" + file_name + ".jpg", img_aruco)