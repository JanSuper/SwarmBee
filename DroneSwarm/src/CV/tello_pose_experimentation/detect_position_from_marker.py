# code based on DroneBlocks-Tello-Python-OpenCV-ArUco-Markers project:
# https://github.com/dbaldwin/DroneBlocks-Tello-Python-OpenCV-ArUco-Markers.git
# this script is a modified version of the "6_display_marker_attitude.py" file

import threading

import cv2
import cv2.aruco as aruco
import os
import pickle
import numpy as np
import pandas as pd
import math

from DroneSwarm.src.Localization.mapping import Plot
from lib.tello import Tello
import time
from DroneSwarm.src.Localization.Localisation import Localiser

# Marker positions on floor (each marker has 50cm in between):

#   0   1   2   3   4   5   36  37  38  39  40  41
#   6   7   8   9  10  11   42  43  44  45  46  47
#  12  13  14  15  16  17   48  49  50  51  52  53
#  18  19  20  21  22  23   54  55  56  57  58  59
#  24  25  26  27  28  29   60  61  62  63  64  65
#  30  31  32  33  34  35   66  67  68  69  70  71

### Marker positions in room (floor configuration)###
# WRONG!!!

#  0----4----3
#  -----------
#  7----8----5
#  -----------
#  1----6----2

# Marker 0 is the origin of coordinate system for
# distance between corners EDGE (measured from one marker's left top corner to the other's left top)
# so distance between marker 0 and 1 is EDGE along x-axis; between marker 0 and 3 is EDGE along y-axis etc.
# and distance between marker 0 and 4 is EDGE/2 along x-axis; between marker 0 and 7 is EDGE/2 along y-axis etc.
# (actual drone coordinates are taken relative to the center of the marker)

EDGE = 200  # 200 cm is the distance between markers in the corners on the floor
EDGE_WALL = 150  # 150 cm is the distance between markers in the corners on the wall

# Marker IDs used:
markers = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]

# temp aruco locations on dec. 6:
# height of marker 0 and 3 center: 170 cm
# distance between marker centers: 154 cm (for marker 0 and 3; 1 and 2)
# height of marker 1 and 2 center: 100 cm

# x and y offset for each marker:
offsetsFloor = [[0, 0], [0, EDGE], [EDGE, EDGE], [EDGE, 0],
           [EDGE/2, 0], [EDGE, EDGE/2], [EDGE/2, EDGE], [0, EDGE/2], [EDGE/2, EDGE/2]]
offsetsWall = [[0, 0], [0, EDGE_WALL], [EDGE_WALL, EDGE_WALL], [EDGE_WALL, 0],
                [EDGE_WALL/2, 0], [EDGE_WALL, EDGE_WALL/2], [EDGE_WALL/2, EDGE_WALL],
               [0, EDGE_WALL/2], [EDGE_WALL/2, EDGE_WALL/2]]
# offsetsTemp = [[0, 0], [0, 60], [EDGE_WALL, 60], [EDGE_WALL, 0],
#              [EDGE_WALL/2, 0], [EDGE_WALL, EDGE_WALL/2], [EDGE_WALL/2, EDGE_WALL],
#              [0, EDGE_WALL/2], [EDGE_WALL/2, EDGE_WALL/2]]
offsetsTemp = [[0, 170], [0, 100], [154, 100], [154, 170],
               [EDGE_WALL/2, 0], [EDGE_WALL, EDGE_WALL/2], [EDGE_WALL/2, EDGE_WALL],
               [0, EDGE_WALL/2], [EDGE_WALL/2, EDGE_WALL/2]]

# TODO MODIFY THIS !!!
OFFSETSprev = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,100],[154,100],[154,0]]

OFFSETS = [[0,0], [50,0], [100,0], [150,0], [200,0],
           [],[],[],[0,0],[0,100],
           [154,100],[154,0],[],[],[],
           [],[],[],[],[],
           [0,0],[0,100],[154,100],[154,0],[]]


# Construct a Tello instance so we can communicate with it over UDP
tello = Tello()
drone_localizer = Localiser()
plot = Plot()

# Send the command string to wake Tello up
tello.send("command")
# tello.send("downvision 1")

# Delay
time.sleep(1)

# Initialize the video stream which will start sending to port 11111
tello.send("streamoff")
tello.send("streamon")

# tello.send("ccw 90")

# Get Tello stream
cap = cv2.VideoCapture('udp://127.0.0.1:11111')  # 0.0.0.0:11111 or 127.0.0.1:11111

#cap = cv2.VideoCapture('./utils/20191219-104330.avi') # Tello video from stream

# Set the camera size - must be consistent with size of calibration photos
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)  # 320 or 960
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)  # 240 or 720

# Specify marker length and params
marker_length = 18
aruco_params = aruco.DetectorParameters_create()
aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

# Font for displaying text on screen
font = cv2.FONT_HERSHEY_SIMPLEX

# prev_marker = -1

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

# tello.send("takeoff")

# for saving received locations in csv
# x_list = []
# y_list = []
# z_list = []
# yaw_list = []

def arloop():
    while True:

        # Read the camera frame
        ret, frame = cap.read()

        img_aruco = frame

        # Convert to gray scale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Find all the aruco markers in the image
        corners, ids, _ = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=aruco_params, cameraMatrix=camera_matrix, distCoeff=distortion_coefficients)

        marker_id = -1
        diff_marker = False
        x = -10000
        y = -10000
        z = -10000
        drone_x = 10000
        drone_y = 10000
        drone_z = 10000

        # Detect ID specified above
        if ids is not None:

            found_marker = ids[0][0]

            # Let's find one of the markers in the room (used marker ids: 0 ... 11)
            if found_marker in range(0, 12):

                marker_id = found_marker

                # Draw the marker boundaries
                img_aruco = aruco.drawDetectedMarkers(frame, corners, ids, (0, 255, 0))

                # Get the marker pose
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, distortion_coefficients)

                # Unpack the rotation and translation values
                rvec = rvec[0, 0, :]
                tvec = tvec[0, 0, :]

                # Plot a point at the center of the image
                # TODO
                cv2.circle(img_aruco, (480, 360), 2, (255, 255, 255), -1)

                # Draw x (red), y (green), z (blue) axes
                img_aruco = aruco.drawAxis(img_aruco, camera_matrix, distortion_coefficients, rvec, tvec, marker_length)

                # Draw black background for text
                cv2.rectangle(img_aruco, (0, 600), (800, 720), (0, 0, 0), -1)

                # save previous read location if now getting position relative to a different marker
                # NOT USING THIS PART FOR LOCALIZATION
                # if diff_marker:
                    # prev_x = drone_x
                    # prev_y = drone_y
                    # prev_z = drone_z

                # marker position relative to drone
                # Display the xyz position coordinates
                x = tvec[0]
                y = tvec[1]
                z = tvec[2]
                #position = "Marker %d position: x=%4.0f y=%4.0f z=%4.0f"%(marker_id, x, y, z)
                #cv2.putText(frame, position, (20, 650), font, 1, (255, 255, 255), 1, cv2.LINE_AA)

                # drone white dot position in relation to the marker
                # check if offsets are taken from wall or floor or temporary setup
                # NOT USING THIS PART FOR LOCALIZATION
                drone_x = OFFSETSprev[marker_id][0] - x  # with coordinating  # TODO pick correct offsets list
                drone_y = OFFSETSprev[marker_id][1] - y  # with coordinating
                drone_z = z  # for some reason height is fine without negation


                # position = "Drone pos. (id %d): x=%4.0f y=%4.0f z=%4.0f"%(marker_id, drone_x, drone_y, drone_z)
                # cv2.putText(frame, position, (20, 650), font, 1, (255, 255, 255), 1, cv2.LINE_AA)

                # check changed position difference / error when reading from a different marker in this frame
                # NOT USING THIS PART FOR LOCALIZATION
                # if diff_marker:
                    # error_x = prev_x - drone_x
                    # error_y = prev_y - drone_y
                    # error_z = prev_z - drone_z
                    # if abs(error_x) > 10 or abs(error_y) > 10 or abs(error_z) > 10:
                        # print("Error > 10 when switching between markers for relative position")

                # Create empty rotation matrix
                rotation_matrix = np.zeros(shape=(3,3))

                # Convert rotation vector to rotation matrix
                cv2.Rodrigues(rvec, rotation_matrix, jacobian = 0)

                # Get yaw/pitch/roll of rotation matrix
                # We are most concerned with rotation around pitch axis which translates to Tello's yaw
                ypr = cv2.RQDecomp3x3(rotation_matrix)

                #print([x, y, z, ypr[0][1]])
                # [dx,dy,dz,dyaw] = drone_localizer.calcPosWallX(x, y, z, ypr[0][1], marker_id)
                #print([dx,dy,dz,dyaw])
                # x_list.append(dx)
                # y_list.append(dy)
                # z_list.append(dz)
                # yaw_list.append(dyaw)

                # Display drone's position in space, and yaw relative to recognized marker
                # position = "Drone pos. (id %d): x=%4.0f y=%4.0f z=%4.0f yaw=%4.0f"%(marker_id, dx, dy, dz, dyaw)
                position = "Drone pos. (id %d): x=%4.0f y=%4.0f z=%4.0f"%(marker_id, drone_x, drone_y, drone_z)
                cv2.putText(frame, position, (20, 620), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
                # TODO do we need to display marker attitude ?
                # Display the yaw/pitch/roll angles
                attitude2 = "Marker %d attitude: y=%4.0f p=%4.0f r=%4.0f"%(marker_id, ypr[0][0], ypr[0][1], ypr[0][2])
                cv2.putText(frame, attitude2, (20, 660), font, 1, (255, 255, 255), 1, cv2.LINE_AA)

                # plot.updateGraph(dx, dy, dz, dyaw, 0)

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


receiveThread = threading.Thread(target=arloop())
receiveThread.daemon = True
receiveThread.start()
#
# tello.send("takeoff")
#tello.send("up 70")
# tello.send("up 50")
# tello.send("ccw 90")
