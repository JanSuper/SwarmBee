import time
import numpy as np
from DroneSwarm.src.Control.Trapezoid import Trapezoid
from DroneSwarm.src.Control.Circle import Circle
from DroneSwarm.src.Utilities.tello_bluetooth_receiver import BackgroundBluetoothSensorRead


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


# Function to check if there exists a value of list1 in interval (lower,upper)
# If there exists a value within interval return True, else return False.
def check_for_interval(list1, lower, upper):
    # traverse in the list
    for x_bar in list1:
        # compare with all the
        # values with value
        if lower < x_bar < upper:
            return False
    return True


class FlightPathController:

    def __init__(self, drone, flightpath, offset, bt_threshold=0.01, interval=0.1):
        self.drone = drone
        self.flightpath = flightpath
        self.offset = offset
        self.bluetooth = BackgroundBluetoothSensorRead()
        self.bt_threshold = bt_threshold
        self.interval = interval
        self.bluetooth.start()
        self.trapezoid = Trapezoid()
        self.circle = Circle()
        self.initial_position = np.zeros(4)  # TODO: Fetch initial_position from first ArUco frame
        self.trapezoid.set_position(self.initial_position)
        if len(flightpath) > 0:
            initial_target = self.initial_position + self.flightpath.pop(0)
            self.completed_flightpath = False
        else:
            initial_target = self.initial_position
            self.completed_flightpath = True
        self.trapezoid.set_target(initial_target)
        time.sleep(3)  # Required for bluetooth values to start coming in

    def safe_for_takeoff(self):
        while not check_for_less(self.bluetooth.current_package, 0.001):
            print("Error: the drone is not in a safe location. Please move the drone.")
            time.sleep(5)
        print("The drone is in a safe location. Please stay clear of the drone.")
        time.sleep(10)

    def fly_trapezoid(self):
        previous_time = time.time()
        while True:
            now = time.time()
            dt = now - previous_time
            if dt > self.interval:
                u = [0, 0, 0, 0]
                if self.drone.leader_drone is None:
                    if not self.completed_flightpath:
                        if self.trapezoid.reached:
                            if len(self.flightpath) > 0:
                                self.trapezoid.set_target(self.initial_position + self.flightpath.pop(0))
                            else:
                                self.completed_flightpath = True
                        else:
                            self.trapezoid.set_position(np.zeros(4))  # TODO: Update Trapezoid's position with ArUco
                            u = self.distance_check_and_calc(self.bt_threshold, method='Trapezoid')
                else:
                    if not (self.completed_flightpath or self.drone.leader_drone.controller.completed_flightpath):
                        if self.trapezoid.reached:
                            if len(self.flightpath) > 0:
                                self.trapezoid.set_target(self.initial_position + self.flightpath.pop(0))
                            else:
                                self.completed_flightpath = True
                        else:
                            self.trapezoid.set_position(np.zeros(4))  # TODO: Update Trapezoid's position with ArUco
                            u = self.distance_check_and_calc(self.bt_threshold, method='Trapezoid')
                    else:
                        if not self.completed_flightpath:
                            self.completed_flightpath = True
                self.drone.send_rc(u)
                previous_time = now

    def fly_circle(self, radius=100, speed=20, clock_wise=True):
        previous_time = time.time()
        while True:
            now = time.time()
            dt = now - previous_time
            if dt > self.interval:
                u = self.distance_check_and_calc(0.01, method='Circle')
                self.drone.send_rc(u)

    # method that checks the distance sensors for values outside 'safe range' and calculates control command
    # method = 'Trapezoid', 'Circle'
    def distance_check_and_calc(self, bt_threshold, method='Trapezoid'):

        if not self.bluetooth.acceptL:
            if check_for_interval(self.bluetooth.current_package[1:2], 0.0, bt_threshold):
                u = self.calc(method)
            else:
                u = [0, 0, 0, 0]
        elif not self.bluetooth.acceptF:
            if check_for_interval(self.bluetooth.current_package[0:2:2], 0.0, bt_threshold):
                u = self.calc(method)
            else:
                u = [0, 0, 0, 0]
        elif not self.bluetooth.acceptR:
            if check_for_interval(self.bluetooth.current_package[0:1], 0.0, bt_threshold):
                u = self.calc(method)
            else:
                u = [0, 0, 0, 0]
        elif self.bluetooth.accept:
            if check_for_interval(self.bluetooth.current_package, 0.0, bt_threshold):
                u = self.calc(method)
            else:
                u = [0, 0, 0, 0]
        else:
            # we do not have anything around and sensors are reading random values
            u = self.calc(method)
        return u

    # Switch between different flight path routines
    def calc(self, method='Trapezoid'):
        match method:
            case 'Trapezoid':
                return self.trapezoid.calculate()
            case 'Circle':
                return self.circle.calculate_angular_velocity()