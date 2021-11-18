import time
import numpy as np

import DroneSwarm.src.Utilities.KeyPressModule as kp
from DroneSwarm.src.Control.Trapezoid import Trapezoid
from DroneSwarm.src.Utilities.utils import *
from DroneSwarm.src.Utilities.tello_bluetooth_receiver import BackgroundBluetoothSensorRead

is_flying = False
# initialize emergency keyboard module
kp.init()
# connect to drone
myDrone = initialize_tello()
# initialize bluetooth thread
bluetooth = BackgroundBluetoothSensorRead()
bluetooth.start()
# initialize Trapezoid controller
trapezoid = Trapezoid()
# it fairly takes 15 seconds to receive first distance value
time.sleep(5)
# takeoff
if is_flying:
    myDrone.takeoff()

# set first desired position
target = np.array([0, 0, 0, 0], dtype=int)
trapezoid.set_target(target)


# Function to check if list1 contains a value less than val
def check_for_less(list1, val):
    # traverse in the list
    for x in list1:
        # compare with all the
        # values with value
        if val <= x:
            return False
    return True


# TODO: run controller on separate thread
interval = 0.25  # 25 ms
previous_time = time.time()
query_time = time.time()
while True:
    now = time.time()
    dt = now - previous_time
    if dt > interval:
        if check_for_less(bluetooth.current_package, 0.5):
            u = trapezoid.calculate()
        else:
            u = [0, 0, 0, 0]
        if kp.get_key("q"):
            myDrone.land()
            break
        if kp.get_key("f"):
            myDrone.emergency()
            break
        # Send desired velocity values to the drone
        if myDrone.send_rc_control:
            myDrone.send_rc_control(*u)
        # Use the real time velocities of the drone
        y = [myDrone.get_speed_x(), myDrone.get_speed_y(), myDrone.get_speed_z(), myDrone.get_yaw()]
        trapezoid.update_position_estimate(dt, *u)
        print("Position: ", trapezoid.position, "Target: ", trapezoid.target, "Time: ", now - query_time, "\n",
              "Control: ", u, "Read: ", y, "Bluetooth: ", bluetooth.current_package)
        img = tello_get_frame(myDrone)
        cv2.imshow('Image', img)
        previous_time = now
