import math
import cv2
import time
import numpy as np
import imutils
# import HandTrackingModule as htm

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

previousTime = 0

# detector = htm.handDetector(detectionCon=0.75)

while True:
    success, img = cap.read()
    img = imutils.resize(img, height=480)

    # img = detector.findHands(img)
    # lmList, bbox = detector.findPosition(img, draw=True)
    # if len(lmList) != 0:
    #     fingers = detector.fingersUp()
    #     print(fingers)
    #     gesture = detector.gestureFinder()
    #     cv2.putText(img, f'{gesture}', (bbox[0], bbox[3]+60), cv2.FONT_HERSHEY_COMPLEX,
    #                 1, (0, 0, 255), 3)

    currentTime = time.time()
    fps = 1 / (currentTime - previousTime)
    previousTime = currentTime

    cv2.putText(img, f'FPS: {int(fps)}', (40, 50), cv2.FONT_HERSHEY_COMPLEX,
                1, (0, 0, 255), 3)
    cv2.imshow("img", img)
    cv2.waitKey(1)
