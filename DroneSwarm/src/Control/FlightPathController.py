import time
import numpy as np
import pandas as pd
from datetime import datetime
import DroneSwarm.src.Utilities.KeyPressModule as kp
from DroneSwarm.src.Control.Trapezoid import Trapezoid
from DroneSwarm.src.Utilities.utils import *
from DroneSwarm.src.Utilities.tello_bluetooth_receiver import BackgroundBluetoothSensorRead
from DroneSwarm.src.Localization.mapping import Plot
import threading

is_flying = True
# initialize emergency keyboard module
kp.init()
# plot values in Realtime
# plot = Plot()
# connect to drone
myDrone = initialize_tello()
# initialize bluetooth thread
bluetooth = BackgroundBluetoothSensorRead()
bluetooth.start()
# initialize Trapezoid controller
trapezoid = Trapezoid()
# it fairly takes 3 seconds to receive first distance value from bluetooth
time.sleep(3)


# Function to check if all the values of list1 are greater than val
# If there exists a values less than val return False, else return True.
def check_for_less(list1, val):
    # traverse in the list
    for x_bar in list1:
        # compare with all the
        # values with value
        if x_bar <= val:
            return False
    return True


# Function to check if there exists a value of list1 are in interval (lower,upper)
# If there exists a value within interval return True, else return False.
def check_for_interval(list1, lower, upper):
    # traverse in the list
    for x_bar in list1:
        # compare with all the
        # values with value
        if lower < x_bar < upper:
            return False
    return True


# takeoff
if is_flying:
    if check_for_less(bluetooth.current_package, -1.0):
        myDrone.takeoff()
    else:
        while True:
            time.sleep(1)
            if check_for_less(bluetooth.current_package, -1.0):
                myDrone.takeoff()
                break
pos, tar, tim, control, sensor, bt = [], [], [], [], [], []
# set first desired position
target = np.array([100, 200, 0, 0], dtype=int)
trapezoid.set_target(target)

# TODO: run controller on separate thread
interval = 0.2  # 10 ms
previous_time = time.time()
query_time = time.time()
while True:
    now = time.time()
    dt = now - previous_time
    if dt > interval:
        if check_for_interval(bluetooth.current_package, 0.0, 0.31):
            u = trapezoid.calculate()
        else:
            u = [0, 0, 0, 0]
        if kp.get_key("q"):
            # datetime object containing current date and time
            now1 = datetime.now()
            # dd/mm/YY H:M:S
            dt_string = now1.strftime("%d/%m/%Y %H:%M:%S")
            df = pd.DataFrame([tim, pos, tar, sensor, control, bt]).T
            df.columns = ['time', 'pos', 'target', 'sensor', 'control', 'bluetooth']
            # pd.set_option('display.max_columns', 4)
            df.to_csv('BluetoothFlightControl.csv')
            myDrone.land()

            # plot.endGraph()
            break
        if kp.get_key("f"):
            myDrone.emergency()
            # plot.endGraph()
            break
        # Send desired velocity values to the drone
        # if myDrone.send_rc_control:
        #     myDrone.send_rc_control(*u)
        myDrone.send_rc_control(*u)
        # Use the real time velocities of the drone
        y = [myDrone.get_speed_x(), myDrone.get_speed_y(), myDrone.get_speed_z(), myDrone.get_yaw()]
        trapezoid.update_position_estimate(dt, *u)
        print("Position: ", trapezoid.position, "Target: ", trapezoid.target, "Time: ", now - query_time, "\n",
              "Control: ", u, "Read: ", y, "Bluetooth: ", bluetooth.current_package)
        pos.append(trapezoid.position)
        tar.append(trapezoid.target)
        tim.append(now - query_time)
        control.append(u)
        sensor.append(y)
        bt.append(bluetooth.current_package)
        # plot.updateGraph(*u, 0)
        # img = tello_get_frame(myDrone)
        # cv2.imshow('Image', img)
        # print("Active Thread: ", threading.activeCount())
        # @TODO: Ask for new target if all target positions are reached
        # if trapezoid.reached:
        #     x, y, z, yaw = [int(a) for a in input("Enter x, y, z, yaw :").split(',')]
        #     target = [x, y, z, yaw]
        #     print("Given target position : ", target)
        #     trapezoid.set_target(target)
        previous_time = now
