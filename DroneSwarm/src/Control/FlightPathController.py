import time
import numpy as np
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
# If there exists a value within interval return True, else return False.
def check_for_interval(list1, lower, upper):
    # traverse in the list
    for x_bar in list1:
        # compare with all the
        # values with value
        if lower < x_bar < upper:
            return False
    return True


# calculate the angular velocity given the
def calculate_angular_velocity(speed, clock_wise, radius):
    control = np.zeros(4, dtype=int)
    if clock_wise:
        control[0] = -1 * speed
    else:
        control[0] = speed
    angular_velocity = -1 * round(control[0]/radius)
    control[3] = angular_velocity
    return control


class FlightPathController:

    def __init__(self, drone, initial_target=np.zeros(4)):
        self.drone = drone
        self.bluetooth = BackgroundBluetoothSensorRead()
        self.bluetooth.start()
        self.trapezoid = Trapezoid()
        initial_position = np.zeros(4)  # TODO: Fetch initial_position from first ArUco frame
        self.trapezoid.set_position(initial_position)
        self.trapezoid.set_target(initial_target + initial_position)
        time.sleep(3)  # Required for bluetooth values to start coming in

    def safe_for_takeoff(self):
        while not check_for_less(self.bluetooth.current_package, 0.001):
            print("Error: the drone is not in a safe location. Please move the drone.")
            time.sleep(5)
        print("The drone is in a safe location. Please stay clear of the drone.")
        time.sleep(10)

    def fly_trapezoid(self):
        bt_threshold = 0.01
        interval = 0.1  # 100 ms
        previous_time = time.time()
        while True:
            now = time.time()
            dt = now - previous_time
            if dt > interval:
                # TODO: Update Trapezoid's position with ArUco
                self.trapezoid.set_position(np.zeros(4))
                if not self.bluetooth.acceptL:
                    if check_for_interval(self.bluetooth.current_package[1:2], 0.0, bt_threshold):
                        u = self.trapezoid.calculate()
                    else:
                        u = [0, 0, 0, 0]
                elif not self.bluetooth.acceptF:
                    if check_for_interval(self.bluetooth.current_package[0:2:2], 0.0, bt_threshold):
                        u = self.trapezoid.calculate()
                    else:
                        u = [0, 0, 0, 0]
                elif not self.bluetooth.acceptR:
                    if check_for_interval(self.bluetooth.current_package[0:1], 0.0, bt_threshold):
                        u = self.trapezoid.calculate()
                    else:
                        u = [0, 0, 0, 0]
                elif self.bluetooth.accept:
                    if check_for_interval(self.bluetooth.current_package, 0.0, bt_threshold):
                        u = self.trapezoid.calculate()
                    else:
                        u = [0, 0, 0, 0]
                else:
                    # we do not have anything around and sensors are reading random values
                    u = self.trapezoid.calculate()
                self.drone.send_rc_command(u)

    def fly_circle(self, radius=100, speed=20, clock_wise=True):
        while True:
            # TODO check sensor read with threshold to see if we have to stop
            # if distance_check()
            control = calculate_angular_velocity(speed, clock_wise, radius)
            self.drone.send_rc(control)


    # TODO make bluetooth threshold check into a function so other functions can make easy call
    # def distance_check(self):
