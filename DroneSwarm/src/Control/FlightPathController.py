import time
import numpy as np
import threading
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


class FlightPathController(threading.Thread):
    def __init__(self, target_queue, queue_lock, my_drone, bluetooth, interval):
        super(FlightPathController, self).__init__()
        # initialize Trapezoid controller
        self.trapezoid = Trapezoid()
        # it fairly takes 15 seconds to receive first distance value
        self.interval = interval  # 10 ms
        self.my_drone = my_drone
        # set first desired position
        self.target_queue = target_queue
        self.trapezoid.set_target(target_queue)
        self.bluetooth = bluetooth
        self.queue_lock = queue_lock

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
        previous_time = time.time()
        query_time = time.time()
        while self.my_drone.is_flying:
            now = time.time()
            dt = now - previous_time
            if dt > self.interval:
                if check_for_less(self.bluetooth.current_package, 0.15):
                    u = self.trapezoid.calculate()
                else:
                    u = [0, 0, 0, 0]
                # Send desired velocity values to the drone
                if self.my_drone.send_rc_control:
                   self.my_drone.send_rc_control(*u)
                # Use the real time velocities of the drone
                y = [self.my_drone.get_speed_x(), self.my_drone.get_speed_y(), self.my_drone.get_speed_z(), self.my_drone.get_yaw()]
                self.trapezoid.update_position_estimate(dt, *u)
                print("Position: ", self.trapezoid.position, "Target: ", self.trapezoid.target, "Time: ", now - query_time, "\n",
                      "Control: ", u, "Read: ", y, "Bluetooth: ", self.bluetooth.current_package)
                # pos.append(self.trapezoid.position)
                # tar.append(self.trapezoid.target)
                # tim.append(now - query_time)
                # control.append(u)
                # sensor.append(y)
                # bt.append(bluetooth.current_package)
                # plot.updateGraph(*u, 0)
                # img = tello_get_frame(self.my_drone)
                # cv2.imshow('Image', img)
                # print("Active Thread: ", threading.activeCount())
                # @TODO: Ask for new target if all target positions are reached
                for i in self.trapezoid.is_reached:
                    if not i:
                        flag = 1
                    else:
                        flag = 0
                        print('Target Reached')
                if flag == 0:
                    x, y, z, yaw = [int(a) for a in input("Enter x, y, z, yaw :").split(',')]
                    target = [x, y, z, yaw]
                    print("Given target position : ", target)
                    self.trapezoid.set_target(self.target_queue.get())
                previous_time = now


