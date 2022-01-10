import time
import numpy as np
from DroneSwarm.src.Control.Circle import Circle
from DroneSwarm.src.Control.Trapezoid import Trapezoid
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
# If there exists a value within interval return False, else return True.
def check_for_interval(list1, lower, upper):
    # traverse in the list
    for x_bar in list1:
        # compare with all the
        # values with value
        if lower < x_bar < upper:
            return False
    return True


class FlightPathController:

    def __init__(self, drone, initial_position, bt_threshold=0.20, interval=0.1):
        self.drone = drone
        self.flightpath = drone.flightpath
        self.need_new_flightpath = False
        self.offset = drone.offset  # offset to the leader = [dx, dy, dz, dyaw]
        # self.circle = Circle()
        self.trapezoid = Trapezoid()
        self.initial_position = initial_position
        self.trapezoid.set_position(self.initial_position)
        if len(self.flightpath) > 0:
            # Flightpath contains an initial target
            initial_target = self.initial_position + np.array(
                self.flightpath.pop(0))  # Map initial target to drone's coordinate system
            self.completed_flightpath = False
        else:
            # Flightpath does not contain an initial target
            # Therefore, initial target is the drone's initial position (stationary)
            initial_target = self.initial_position
            self.completed_flightpath = True
        self.trapezoid.set_target(initial_target)
        self.need_new_position = False
        # Initialize Bluetooth
        self.bluetooth = BackgroundBluetoothSensorRead(drone.bt_address)
        self.bt_threshold = bt_threshold
        self.bluetooth.start()
        time.sleep(3)  # Takes roughly three seconds before Bluetooth values start coming in
        self.interval = interval

    # Function that checks whether it is safe for the drone to perform takeoff
    def check_safe_for_takeoff(self):
        while not check_for_less(self.bluetooth.current_package, 0.001):
            print("Error: the drone is not in a safe location. Please move the drone.")
            time.sleep(5)
        print("The drone is in a safe location. Please stay clear of the drone.")
        time.sleep(10)

    # Function that flies the drone by using the Trapezoid controller
    def fly_trapezoid(self):
        previous_time = time.time()
        while True:
            now = time.time()
            dt = now - previous_time
            if dt > self.interval:
                u = [0, 0, 0, 0]  # Assume the drone is to be stationary; these values will only change if the drone
                # has not reached its current target yet
                if not self.completed_flightpath:
                    calculate_u = False
                    # Drone has not completed its flightpath yet
                    if self.trapezoid.reached:
                        # Drone has reached its current target
                        if len(self.flightpath) > 0:
                            # Flightpath contains at least one more target; update drone's target
                            self.trapezoid.set_target(self.initial_position + np.array(self.flightpath.pop(0)))
                            calculate_u = True
                        else:
                            # Flightpath has been completed
                            self.completed_flightpath = True
                    else:
                        # Drone has not reached its current target
                        calculate_u = True
                    if calculate_u:
                        # Update drone's current position
                        self.update_drone_position()
                        # Calculate velocities using the Trapezoid controller
                        u = self.distance_check_calculate_u(self.bt_threshold, method='Trapezoid')
                # Send velocities to drone
                self.drone.send_rc(u)
                previous_time = now

    # Function that flies the drone in a circle
    # TODO: add leader-follower logic
    def fly_circle(self, radius=100, speed=20, clock_wise=True):
        previous_time = time.time()
        while True:
            now = time.time()
            dt = now - previous_time
            if dt > self.interval:
                u = self.distance_check_calculate_u(0.01, method='Circle')
                self.drone.send_rc(u)

    # Function that request an external module to update the drone's current position
    def update_drone_position(self):
        # Send request to external module
        self.need_new_position = True
        # Wait for a response
        while self.need_new_position:
            pass

    # Function that checks the distance sensors for values outside 'safe range' and calculates control command
    # method = 'Trapezoid', 'Circle'
    def distance_check_calculate_u(self, bt_threshold, method='Trapezoid'):
        u = [0, 0, 0, 0]  # Assume the drone is to be stationary
        # Perform distance check and calculate velocities
        if not self.bluetooth.acceptL:
            if check_for_interval([self.bluetooth.current_package[0]], 0.0, bt_threshold):
                u = self.calculate_u(method)
        elif not self.bluetooth.acceptF:
            if check_for_interval([self.bluetooth.current_package[1]], 0.0, bt_threshold):
                u = self.calculate_u(method)
        elif not self.bluetooth.acceptR:
            if check_for_interval([self.bluetooth.current_package[2]], 0.0, bt_threshold):
                u = self.calculate_u(method)
        elif self.bluetooth.accept:
            if check_for_interval(self.bluetooth.current_package, 0.0, bt_threshold):
                u = self.calculate_u(method)
        else:
            # We do not have anything around and sensors are reading random values
            u = self.calculate_u(method)
        u[2], u[3] = 0, 0  # TODO: remove once Trapezoid is more stable
        return u

    # Function that allows to switch between different flight path routines
    def calculate_u(self, method='Trapezoid'):
        match method:
            case 'Trapezoid':
                return self.trapezoid.calculate()
            case 'Circle':
                return self.circle.calculate_angular_velocity()
