# code based on DroneBlocks-Tello-Python-OpenCV-ArUco-Markers project:
# https://github.com/dbaldwin/DroneBlocks-Tello-Python-OpenCV-ArUco-Markers.git

import numpy
import cv2
from cv2 import aruco

# ChAruco board variables
CHARUCOBOARD_ROWCOUNT = 7
CHARUCOBOARD_COLCOUNT = 5 
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

# Create constants to be passed into OpenCV and Aruco methods
board = CHARUCO_BOARD = aruco.CharucoBoard_create(
        squaresX=CHARUCOBOARD_COLCOUNT,
        squaresY=CHARUCOBOARD_ROWCOUNT,
        squareLength=0.04,  # should be good for A4 paper
        markerLength=0.02,  # needs to be half of squareLength
        dictionary=ARUCO_DICT)

image = board.draw((1000, 1000))
cv2.imwrite("charuco_board.png", image)
