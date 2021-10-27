from DroneFaceRec.src import utils
from DroneFaceRec.src.utils import *
from DroneFaceRec.src.PID import PID
import time

#Set camera view window size
w, h = 360, 240

#Create lists to save current locations to display them later
currentX = [0]
currentY = [0]
currentZ = [0]

#connect to drone
myDrone = initializeTello()

#initialise PID
PIDcontrol = PID(myDrone) #

#takeoff
myDrone.takeoff()

#set first desired position
PIDcontrol.setDesiredPos(0, 0, 8, 0)

#record current time for eulers method
previousTime = time.time()

while True:
    print("Position:", PIDcontrol.currentX, PIDcontrol.currentY, PIDcontrol.currentZ)

    #Do next continuous PID calculation
    PIDcontrol.calcVel()

    #Send desired velocity values to the drone
    if myDrone.send_rc_control:
        myDrone.send_rc_control(int(PIDcontrol.Xvel),
                                int(PIDcontrol.Yvel),
                                int(PIDcontrol.Zvel),
                                int(PIDcontrol.Yawvel))

    # Use the real time speed values of the drone as it might be a bit different than the desires ones
    VelX = myDrone.get_speed_x()
    VelY = myDrone.get_speed_y()
    VelZ = myDrone.get_speed_z()

    # Get actual yaw of drone
    yaw = myDrone.get_yaw()

    #Get the delta time between now and previous time
    #TODO: might be too little time between each drone instruction, so we might wanna do it every x loops
    currentTime = time.time()
    deltaTime = currentTime - previousTime

    #Use eulers method to calculate now position
    #Save the new position in an array so that we can plot it to see the position of the drone in a graph
    #TODO: might also be too many points, so we might wanna change it to recording the position once every x loops
    currentX.append(currentX[len(currentX) - 1] + VelX * deltaTime)
    currentY.append(currentY[len(currentY) - 1] + VelY * deltaTime)
    currentZ.append(currentZ[len(currentZ) - 1] + VelZ * deltaTime)

    #update current real position in the PID controller
    PIDcontrol.setCurrentPos(currentX[len(currentX) - 1], currentY[len(currentY) - 1], currentZ[len(currentZ) - 1], yaw)

    #set previous time for next loop
    previousTime = currentTime

    #grab frame from the drone camera and display it
    myFrame = myDrone.get_frame_read()
    myFrame = myFrame.frame
    img = cv2.resize(myFrame, (w, h))
    cv2.imshow('Image', img)

    #kill the PID controller and make the drone land when you press q
    if (cv2.waitKey(1) & 0xFF == ord('q')):
        myDrone.land()
        break
