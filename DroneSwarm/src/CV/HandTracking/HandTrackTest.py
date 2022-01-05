import math
import cv2
import time
import numpy as np
import imutils
import HandTrackingModule as htm


cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

previousTime = 0

detector = htm.handDetector(detectionCon=0.75)



while True:
    success, img = cap.read()
    img = imutils.resize(img, height=480)

    img = detector.findHands(img)
    lmList, bbox = detector.findPosition(img, draw=True)
    if len(lmList) != 0:

        #Code to find distance between thumb tip and index tip for tutorial purposes

        #posList = lmList
        #print(posList[4],posList[8])

        # x1, y1 = posList[4][1], posList[4][2]
        # x2, y2 = posList[8][1], posList[8][2]
        # cx,cy = (x1+x2)//2, (y1+y2)//2
        #
        # cv2.circle(img, (x1,y1), 15, (255,0,0),cv2.FILLED)
        # cv2.circle(img, (x2, y2), 15, (255, 0, 0), cv2.FILLED)
        # cv2.line(img, (x1,y1),(x2, y2), (255, 0, 0),3)
        # cv2.circle(img, (cx, cy), 15, (255, 0, 0), cv2.FILLED)
        #
        # distance = math.hypot(abs(x1-x2),abs(y1-y2))
        #
        # #distance can change based on distance from camera so it needs to be relative
        # x1, y1 = posList[1][1], posList[1][2]
        # x2, y2 = posList[2][1], posList[2][2]
        # distance = distance/math.hypot(abs(x1 - x2), abs(y1 - y2))
        # print(distance)

        fingers = detector.fingersUp()
        print(fingers)
        gesture = detector.gestureFinder()
        cv2.putText(img, f'{gesture}', (bbox[0], bbox[3]+60), cv2.FONT_HERSHEY_COMPLEX,
                    1, (0, 0, 255), 3)



    currentTime = time.time()
    fps = 1/(currentTime-previousTime)
    previousTime = currentTime

    cv2.putText(img, f'FPS: {int(fps)}', (40,50), cv2.FONT_HERSHEY_COMPLEX,
                1, (0,0,255),3)
    cv2.imshow("img", img)
    cv2.waitKey(1)