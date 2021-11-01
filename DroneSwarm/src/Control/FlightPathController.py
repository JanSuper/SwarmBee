from DroneSwarm.src.Utilities.utils import *
from DroneSwarm.src.Control.PID import PID
import time

# Set camera view window size
(w, h) = (360, 240)

interval = 0.005  # 5 ms
previous_time = 0

# Create lists to save current locations to display them later

currentX = [0]
currentY = [0]
currentZ = [0]

# connect to drone
myDrone = initializeTello()

# takeoff
myDrone.takeoff()
# TODO: initialize PID when drone completes take-off and hovering
# initialize PID
pid = PID()

# # set first desired position
# pid.setDesiredPos(0, 0, 0, 0)
# TODO: run PID on separate thread
while True:
    now = time.time()
    dt = now - previous_time
    if dt >= interval:
        print("Position: ", pid.position, "Time: ", now)
        u = pid.calculate(dt)
        # Send desired velocity values to the drone
        if myDrone.send_rc_control:
            myDrone.send_rc_control(*u)
        previous_time = now
        # Use the real time speed values of the drone as it might be a bit different than the desires ones
        y = np.array([myDrone.get_speed_x(), myDrone.get_speed_x(), myDrone.get_speed_z(), myDrone.get_yaw()]) * dt
        print("Control: ", u, "Read: ", y)
        pid.update_position(*y)

    # # grab frame from the drone camera and display it
    # myFrame = myDrone.get_frame_read()
    # myFrame = myFrame.frame
    # img = cv2.resize(myFrame, (w, h))
    # cv2.imshow('Image', img)

    # kill the PID controller and make the drone land when you press q
    if cv2.waitKey(1) & 0xFF == ord('q'):
        myDrone.land()
        break
