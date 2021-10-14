from DroneFaceRec.src import utils
from DroneFaceRec.src.utils import *
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import numpy as np
import time

velX = []
velY = []
velZ = []

w, h = 360, 240
pid = [0.4, 0.4, 0]
pError = 0
startCounter = 0  # for no Flight 1   - for flight 0

#time.sleep(10)
myDrone = initializeTello()

startTime = time.time()
while True:
    currentTime = time.time()

    velX.append(myDrone.get_speed_x())
    velY.append(myDrone.get_speed_y())
    velZ.append(myDrone.get_speed_z())

    # Flight
    if startCounter == 0:
        myDrone.takeoff()
        myDrone.move_up(100)
        startCounter = 1

    # Step 1
    img = telloGetFrame(myDrone, w, h)
    # Step 2
    img, info = findFace(img)
    # Step 3
    pError = utils.trackFace(myDrone, info, w, pid, pError)
    #if info[0][0] != 0:
     #   myDrone.flip_forward()

    cv2.imshow('Image', img)
    if currentTime - startTime >= 80 or (cv2.waitKey(1) & 0xFF == ord('q')):
        myDrone.land()
        break

fig1 = plt.figure()
plot = plt.axes(projection='3d')
plot.plot3D(velX, velY, velZ)
plt.show()
