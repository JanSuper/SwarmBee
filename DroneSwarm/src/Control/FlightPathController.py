import time
import numpy as np
from threading import Thread

import pandas as pd
from datetime import datetime
import DroneSwarm.src.Utilities.KeyPressModule as kp
from DroneSwarm.src.Control.Trapezoid import Trapezoid
from DroneSwarm.src.Utilities.utils import *
# from DroneSwarm.src.Utilities.tello_bluetooth_receiver import BackgroundBluetoothSensorRead
import DroneSwarm.src.Utilities.tello_bluetooth_receiver as BTR

# Function to check if all the values of list1 are greater than val

from DroneSwarm.src.Utilities.tello_bluetooth_receiver import BackgroundBluetoothSensorRead
from DroneSwarm.src.Localization.mapping import Plot
import threading

class FlightPathController(Thread):

    def __init__(self):
        Thread.__init__(self)
        self.is_flying = True
        # initialize emergency keyboard module
        self.kp.init()
        # connect to drone
        self.self.myDrone = initialize_tello()
        # initialize bluetooth thread
        self.bluetooth = BTR.BackgroundBluetoothSensorRead()
        # initialize Trapezoid controller
        self.trapezoid = Trapezoid()
        # it fairly takes 15 seconds to receive first distance value
        time.sleep(5)
        # takeoff
        if self.is_flying:
            self.self.myDrone.takeoff()
        # set first desired position
        self.target = np.array([0, 100, 0, 0], dtype=int)
        self.trapezoid.set_target(self.target)
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
# takeoff
if is_flying:
    myDrone.takeoff()
pos, tar, tim, control, sensor, bt = [], [], [], [], [], []
# set first desired position
target = np.array([0, 150, 0, 0], dtype=int)
trapezoid.set_target(target)


# Function to check if all the values of list1 are greater than val
def check_for_less(list1, val):
    # traverse in the list
    for x in list1:
        # compare with all the
        # values with value
        if x < val:
            return False
    return True


    # TODO: run controller on separate thread
    def run(self):
        interval = 0.1  # 10 ms
        previous_time = time.time()
        query_time = time.time()
        while self.is_flying:
            now = time.time()
            dt = now - previous_time
            if dt > interval:
                if check_for_less(self.bluetooth.current_package, 0.4):
                    u = self.trapezoid.calculate()
                else:
                    u = [0, 0, 0, 0]
                if kp.get_key("q"):
                    self.myDrone.land()
                    break
                if kp.get_key("f"):
                    self.myDrone.emergency()
                    break
                # Send desired velocity values to the drone
                if self.myDrone.send_rc_control:
                    self.myDrone.send_rc_control(*u)
                # Use the real time velocities of the drone
                y = [self.myDrone.get_speed_x(), self.myDrone.get_speed_y(), self.myDrone.get_speed_z(), self.myDrone.get_yaw()]
                self.trapezoid.update_position_estimate(dt, *u)
                print("Position: ", self.trapezoid.position, "Target: ", self.trapezoid.target, "Time: ", now - query_time, "\n",
                      "Control: ", u, "Read: ", y, "Bluetooth: ", self.bluetooth.current_package)
                # img = tello_get_frame(self.myDrone)
                # cv2.imshow('Image', img)
                # print("Active Thread: ", threading.activeCount())
                previous_time = now
# TODO: run controller on separate thread
interval = 0.1  # 10 ms
previous_time = time.time()
query_time = time.time()
while True:
    now = time.time()
    dt = now - previous_time
    if dt > interval:
        if check_for_less(bluetooth.current_package, 0.15):
            u = trapezoid.calculate()
        else:
            u = [0, 0, 0, 0]
        if kp.get_key("q"):
            myDrone.land()
            # datetime object containing current date and time
            now1 = datetime.now()
            # dd/mm/YY H:M:S
            dt_string = now1.strftime("%d/%m/%Y %H:%M:%S")
            df = pd.DataFrame([tim, pos, tar, sensor, control, bt]).T
            df.columns = ['time', 'pos', 'target', 'sensor', 'control', 'bluetooth']
            # pd.set_option('display.max_columns', 4)
            df.to_csv('BluetoothFlightControl.csv')
            # plot.endGraph()
            break
        if kp.get_key("f"):
            myDrone.emergency()
            # plot.endGraph()
            break
        # Send desired velocity values to the drone
        if myDrone.send_rc_control:
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
        for i in trapezoid.is_reached:
            if not i:
                flag = 1
            else:
                flag = 0
                print('Target Reached')
        if flag == 0:
            x, y, z, yaw = [int(a) for a in input("Enter x, y, z, yaw :").split(',')]
            target = [x, y, z, yaw]
            print("Given target position : ", target)
            trapezoid.set_target(target)
        previous_time = now
