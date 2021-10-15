from DroneFaceRec.src import utils
from DroneFaceRec.src.utils import *
from DroneFaceRec.src.PID import PID
import time

w, h = 360, 240

currentX = [0]
currentY = [0]
currentZ = [0]
currentYaw = [0]

myDrone = initializeTello()

PIDcontrol = PID(myDrone) #

PID.setDesiredPos(0, 0, 30, 0)

previousTime = time.time()

while True:
    myFrame = myDrone.get_frame_read()
    myFrame = myFrame.frame
    img = cv2.resize(myFrame, (w, h))

    PIDcontrol.calcVel()
    if myDrone.send_rc_control:
        myDrone.send_rc_control(PIDcontrol.Xvel,
                                PIDcontrol.Yvel,
                                PIDcontrol.Zvel,
                                PIDcontrol.Yawvel)

    VelX = myDrone.get_speed_x()
    VelY = myDrone.get_speed_y()
    VelZ = myDrone.get_speed_z()

    currentTime = time.time()
    deltaTime = currentTime - previousTime

    currentX.append(currentX[currentX.len - 1] + VelX * deltaTime)
    currentY.append(currentX[currentY.len - 1] + VelY * deltaTime)
    currentZ.append(currentX[currentZ.len - 1] + VelZ * deltaTime)

    previousTime = currentTime

    cv2.imshow('Image', img)
    if (cv2.waitKey(1) & 0xFF == ord('q')):
        myDrone.land()
        break
