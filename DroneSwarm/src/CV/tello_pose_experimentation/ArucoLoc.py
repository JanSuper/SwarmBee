# code based on DroneBlocks-Tello-Python-OpenCV-ArUco-Markers project:
# https://github.com/dbaldwin/DroneBlocks-Tello-Python-OpenCV-ArUco-Markers.git
# this script is a modified version of the "6_display_marker_attitude.py" file

import threading

import cv2
import cv2.aruco as aruco
import os
import pickle
import numpy as np
from lib.tello import Tello
import time


# Marker positions on floor (each marker has 50cm in between):

#   0   1   2   3   4   5   36  37  38  39  40  41
#   6   7   8   9  10  11   42  43  44  45  46  47
#  12  13  14  15  16  17   48  49  50  51  52  53
#  18  19  20  21  22  23   54  55  56  57  58  59
#  24  25  26  27  28  29   60  61  62  63  64  65
#  30  31  32  33  34  35   66  67  68  69  70  71

class ArucoLoc:

    a = 50
    markers = []
    for i in range(72):
        markers.append(i)
    print(markers)

    # x and y offsets for each marker:

    OFFSETS = [[0, 0], [a, 0], [2 * a, 0], [3 * a, 0], [4 * a, 0], [5 * a, 0],
               [0, a], [a, a], [2 * a, a], [3 * a, a], [4 * a, a], [5 * a, a],
               [0, 2 * a], [a, 2 * a], [2 * a, 2 * a], [3 * a, 2 * a], [4 * a, 2 * a], [5 * a, 2 * a],
               [0, 3 * a], [a, 3 * a], [2 * a, 3 * a], [3 * a, 3 * a], [4 * a, 3 * a], [5 * a, 3 * a],
               [0, 4 * a], [a, 4 * a], [2 * a, 4 * a], [3 * a, 4 * a], [4 * a, 4 * a], [5 * a, 4 * a],
               [0, 5 * a], [a, 5 * a], [2 * a, 5 * a], [3 * a, 5 * a], [4 * a, 5 * a], [5 * a, 5 * a],
               [6 * a, 0], [7 * a, 0], [8 * a, 0], [9 * a, 0], [10 * a, 0], [11 * a, 0],
               [6 * a, a], [7 * a, a], [8 * a, a], [9 * a, a], [10 * a, a], [11 * a, a],
               [6 * a, 2 * a], [7 * a, 2 * a], [8 * a, 2 * a], [9 * a, 2 * a], [10 * a, 2 * a], [11 * a, 2 * a],
               [6 * a, 3 * a], [7 * a, 3 * a], [8 * a, 3 * a], [9 * a, 3 * a], [10 * a, 3 * a], [11 * a, 3 * a],
               [6 * a, 4 * a], [7 * a, 4 * a], [8 * a, 4 * a], [9 * a, 4 * a], [10 * a, 4 * a], [11 * a, 4 * a],
               [6 * a, 5 * a], [7 * a, 5 * a], [8 * a, 5 * a], [9 * a, 5 * a], [10 * a, 5 * a], [11 * a, 5 * a]]

    # Specify marker length and params
    marker_length = 18
    aruco_params = aruco.DetectorParameters_create()
    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

    # Font for displaying text on screen
    font = cv2.FONT_HERSHEY_SIMPLEX

    def __init__(self, tello):

        # set Tello object
        self.tello = tello
        # Get Tello stream # TODO needs to be modified per drone
        self.cap = cv2.VideoCapture('udp://127.0.0.1:11111')  # 0.0.0.0:11111 or 127.0.0.1:11111
        # Set the camera size - must be consistent with size of calibration photos
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # Load camera calibration data
        if not os.path.exists('./tello_calibration.pckl'):
            print("Missing Tello camera calibration file: tello_calibration.pckl")
            exit()
        else:
            f = open('tello_calibration.pckl', 'rb')
            (self.camera_matrix, self.distortion_coefficients, _, _) = pickle.load(f)
            f.close()
            if self.camera_matrix is None or self.distortion_coefficients is None:
                print("Calibration issue. You may need to recalibrate.")
                exit()

        # # Send the command string to wake Tello up
        # self.tello.send("command")
        #
        # # Delay
        # time.sleep(1)
        #
        # # Initialize the video stream which will start sending to port 11111
        # self.tello.send("streamoff")
        # self.tello.send("streamon")

    def aruco_loop(self):

        drone_x = 0
        drone_y = 0
        drone_z = 0
        yaw = 0

        while True:

            # Read the camera frame
            ret, frame = self.cap.read()
            img_aruco = frame

            # Convert to gray scale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Find all the aruco markers in the image
            corners, ids, _ = aruco.detectMarkers(image=gray, dictionary=ArucoLoc.aruco_dict,
                                                  parameters=ArucoLoc.aruco_params, cameraMatrix=self.camera_matrix,
                                                  distCoeff=self.distortion_coefficients)

            # Detect ID specified above
            if ids is not None:

                found_marker = ids[0][0]

                # Let's find one of the markers in the room (used marker ids: 0 ... 71)
                if found_marker in range(0, 72):  # TODO

                    marker_id = found_marker

                    # Draw the marker boundaries
                    img_aruco = aruco.drawDetectedMarkers(frame, corners, ids, (0, 255, 0))

                    # Get the marker pose
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, ArucoLoc.marker_length,
                                                                    self.camera_matrix, self.distortion_coefficients)

                    # Unpack the rotation and translation values
                    rvec = rvec[0, 0, :]
                    tvec = tvec[0, 0, :]

                    # Plot a point at the center of the image
                    cv2.circle(img_aruco, (480, 360), 2, (255, 255, 255), -1)

                    # Draw x (red), y (green), z (blue) axes
                    img_aruco = aruco.drawAxis(img_aruco, self.camera_matrix, self.distortion_coefficients,
                                               rvec, tvec, ArucoLoc.marker_length)

                    # Draw black background for text
                    cv2.rectangle(img_aruco, (0, 600), (800, 720), (0, 0, 0), -1)

                    # marker position relative to drone
                    # Display the xyz position coordinates
                    x = tvec[0]
                    y = tvec[1]
                    z = tvec[2]

                    # drone white dot position in relation to the marker
                    # check if offsets are taken from wall or floor or temporary setup
                    drone_x = ArucoLoc.OFFSETS[marker_id][0] - x  # with coordinating
                    drone_y = ArucoLoc.OFFSETS[marker_id][1] - y  # with coordinating
                    drone_z = z  # for some reason height is fine without negation

                    # Create empty rotation matrix
                    rotation_matrix = np.zeros(shape=(3, 3))

                    # Convert rotation vector to rotation matrix
                    cv2.Rodrigues(rvec, rotation_matrix, jacobian=0)

                    # Get yaw/pitch/roll of rotation matrix
                    # We are most concerned with rotation around pitch axis which translates to Tello's yaw
                    ypr = cv2.RQDecomp3x3(rotation_matrix)

                    # Display drone's position in space, and yaw relative to recognized marker
                    # position = "Drone pos. (id %d): x=%4.0f y=%4.0f z=%4.0f yaw=%4.0f"%(marker_id, dx, dy, dz, dyaw)
                    position = "Drone pos. (id %d): x=%4.0f y=%4.0f z=%4.0f" % (marker_id, drone_x, drone_y, drone_z)
                    cv2.putText(frame, position, (20, 650), ArucoLoc.font, 1, (255, 255, 255), 1, cv2.LINE_AA)
                    # Display the yaw/pitch/roll angles
                    yaw = ypr[0][2]
                    attitude2 = "Marker %d attitude: y=%4.0f p=%4.0f r=%4.0f" % (
                    marker_id, ypr[0][0], ypr[0][1], ypr[0][2])
                    cv2.putText(frame, attitude2, (20, 690), ArucoLoc.font, 1, (255, 255, 255), 1, cv2.LINE_AA)

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

            return [drone_x, drone_y, drone_z, yaw]

# # test script
# drone = Tello()
# aruco_loc = ArucoLoc(drone)
# # aruco_loc.prep_for_aruco()
# receiveThread = threading.Thread(target=aruco_loc.aruco_loop())
# receiveThread.daemon = True
# receiveThread.start()
