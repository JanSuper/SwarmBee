# main script to run multi-processing
import threading
import queue
import time
import numpy as np
from threading import Thread

import pandas as pd
from datetime import datetime
import DroneSwarm.src.Control
import DroneSwarm.src.Utilities.KeyPressModule as kp
from DroneSwarm.src.Control.FlightPathController import FlightPathController
from DroneSwarm.src.Utilities.utils import *
import DroneSwarm.src.Utilities.tello_bluetooth_receiver as BTR


# def drive(speed_queue):
#     speed = 1
#     while True:
#         try:
#             speed = speed_queue.get(timeout=1)
#             if speed == 0:
#                 break
#         except queue.Empty:
#             pass
#         print("speed:", speed)
#
# def main():
#     speed_queue = queue.Queue()
#     threading.Thread(target=drive, args=(speed_queue,)).start()
#     while True:
#         speed = int(input("Enter 0 to Exit or 1/2/3 to continue: "))
#         speed_queue.put(speed)
#         if speed == 0:
#             break
#
# main()

INTERVAL = 0.1

if __name__ == "__main__":
    # initialize emergency keyboard module
    kp.init()
    # plot values in Realtime
    # plot = Plot()
    # connect to drone
    my_drone = initialize_tello()
    # initialize bluetooth thread
    bluetooth = BTR.BackgroundBluetoothSensorRead()
    target_queue = queue.Queue()
    q_Lock = threading.Lock()
    flight_controller_thread = FlightPathController(target_queue, q_Lock, my_drone, bluetooth, INTERVAL)
    if kp.get_key("q"):
        my_drone.land()
        my_drone.is_flying = False
        # # datetime object containing current date and time
        # now1 = datetime.now()
        # dd/mm/YY H:M:S
        # dt_string = now1.strftime("%d/%m/%Y %H:%M:%S")
        # df = pd.DataFrame([tim, pos, tar, sensor, control, bt]).T
        # df.columns = ['time', 'pos', 'target', 'sensor', 'control', 'bluetooth']
        # # pd.set_option('display.max_columns', 4)
        # df.to_csv('BluetoothFlightControl.csv')
        # # plot.endGraph()
    if kp.get_key("f"):
        my_drone.emergency()
        my_drone.is_flying = False
        # plot.endGraph()
