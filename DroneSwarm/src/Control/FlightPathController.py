import time
import numpy as np
import math

from DroneSwarm.src.Control.Trapezoid import Trapezoid
from DroneSwarm.src.Swarm import autonomous_swarm
from DroneSwarm.src.Utilities.tello_bluetooth_receiver import BackgroundBluetoothSensorRead
from DroneSwarm.src.Control.ArucoPID import APID
from DroneSwarm.src.Control.Circle import Circle


# calculate the angular velocity given the speed and direction to spin in, with a radius of the arc to follow
def calculate_angular_velocity(speed, radius=100):
    control = [speed, 0, 0, 0]
    print(math.degrees(control[0] / radius))
    angular_velocity = -1 * round(math.degrees(control[0] / radius))
    control[3] = angular_velocity
    return control


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

    def __init__(self, drone, initial_position, method='Trapezoid', bt_threshold=0.20, interval=0.1, max_speed=40):
        self.drone = drone
        self.flightpath = drone.flightpath
        self.need_new_flightpath = False
        self.offset = drone.offset  # offset to the leader = [dx, dy, dz, dyaw]

        self.initial_position = initial_position
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

        print(f"Drone #{self.drone.number}: new target {initial_target}")
        autonomous_swarm.current_target = initial_target.tolist()
        match method:
            case 'Trapezoid':
                self.trapezoid = Trapezoid()
                self.trapezoid.set_position(self.initial_position)
                self.trapezoid.set_target(initial_target)
            case 'Proportional':
                self.proportional = APID(initial_target)
            case 'Circle':
                self.circle = Circle(speed=max_speed)

        self.current_position = initial_position
        self.need_new_position = False

        # Initialize Bluetooth
        # if drone.bt_address is not None:
        #     self.bluetooth = BackgroundBluetoothSensorRead(drone.bt_address)
        #     self.bt_threshold = bt_threshold
        #     self.bluetooth.start()
        #     while self.bluetooth.current_package == [0, 0, 0]:
        #         # print(f"(Drone #{drone.number}) Bluetooth values: {self.bluetooth.current_package}")
        #         pass
        #     # print(f"(Drone #{drone.number}) Bluetooth values: {self.bluetooth.current_package}")

        self.flight_interrupted = False
        self.position_before_interruption = None
        self.interval = interval
        self.MAX_SPEED = max_speed
        self.obstacleList = []  # [150, 50], [150, 200]
        self.last_velocity = None

    # Function that checks whether it is safe for the drone to perform takeoff
    def check_safe_for_takeoff(self):
        while not check_for_less(self.bluetooth.current_package, 0.001):
            print("Error: the drone is not in a safe location. Please move the drone.")
            time.sleep(5)
        print("The drone is in a safe location. Please stay clear of the drone.")
        time.sleep(10)

    def update_flightpath(self, flightpath, method='Trapezoid'):
        self.flightpath = flightpath
        next_target = self.current_position + np.array(self.flightpath.pop(0))
        print(f"Drone #{self.drone.number}: new target {next_target}")
        autonomous_swarm.current_target = next_target.tolist()
        match method:
            case 'Trapezoid':
                self.trapezoid.set_target(next_target)
            case 'Proportional':
                self.proportional.setDes(next_target)

    def save_current_position(self):
        current_position = self.current_position
        if self.trapezoid is not None:
            # Trapezoid controller is used
            deltas = self.trapezoid.target - current_position
        else:
            # Proportional controller is used
            deltas = np.array([self.proportional.desx, self.proportional.desy, self.proportional.desz,
                               self.proportional.desyaw]) - current_position
        self.flightpath.insert(0, deltas)
        self.position_before_interruption = current_position

    def restore_position_before_interruption(self):
        if self.trapezoid is not None:
            # Trapezoid controller is used
            self.trapezoid.set_target(self.position_before_interruption)
        else:
            # Proportional controller is used
            self.proportional.setDes(self.position_before_interruption)

    def fly_you_fool(self, method='Trapezoid'):
        # datetime object containing current date and time
        match method:
            case 'Trapezoid':
                self.fly_trapezoid()
            case 'Proportional':
                self.fly_proportional()
            case 'Circle':
                self.fly_circle()

    # Function that flies the drone by using the Trapezoid controller
    def fly_trapezoid(self):
        previous_time = time.time()
        while True:
            now = time.time()
            dt = now - previous_time
            if dt > self.interval:
                u = [0, 0, 0, 0]  # Assume the drone is to be stationary; these values will only change if the drone
                # has not reached its current target yet
                if not self.flight_interrupted:
                    # Hand-tracking module is not active
                    if not self.completed_flightpath:
                        calculate_u = False
                        # Drone has not completed its flightpath yet
                        if self.trapezoid.reached:
                            # Drone has reached its current target
                            if len(self.flightpath) > 0:
                                # Flightpath contains at least one more target; update drone's target
                                new_target = self.current_position + np.array(self.flightpath.pop(0))
                                print(f"Drone #{self.drone.number}: new target {new_target}")
                                autonomous_swarm.current_target = new_target.tolist()
                                self.trapezoid.set_target(new_target)
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
                            self.trapezoid.set_position(self.current_position)
                            # Calculate velocities using the Trapezoid controller
                            u = self.calculate_u(method='Trapezoid')
                            self.last_velocity = u
                    # Send velocities to drone
                    self.drone.send_rc(u)
                previous_time = now

    # Function that flies the drone by using the Proportional controller
    def fly_proportional(self):
        previous_time = time.time()
        while True:
            now = time.time()
            dt = now - previous_time
            if dt > self.interval:
                u = [0, 0, 0, 0]  # Assume the drone is to be stationary; these values will only change if the drone
                # has not reached its current target yet
                if not self.flight_interrupted:
                    # Hand-tracking module is not active
                    if not self.completed_flightpath:
                        calculate_u = False
                        # Drone has not completed its flightpath yet
                        if self.proportional.reachedTarget:
                            # Drone has reached its current target
                            if len(self.flightpath) > 0:
                                # Flightpath contains at least one more target; update drone's target
                                new_target = self.current_position + np.array(self.flightpath.pop(0))
                                print(f"Drone #{self.drone.number}: new target {new_target}")
                                autonomous_swarm.current_target = new_target.tolist()
                                self.proportional.setDes(new_target)
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
                            self.proportional.realUpdate(self.current_position)
                            # Calculate velocities using the Proportional controller
                            u = self.calculate_u(method='Proportional')
                            self.last_velocity = u
                    # Send velocities to drone
                    self.drone.send_rc(u)
                previous_time = now

    # Function that flies the drone in a circle
    # TODO: add leader-follower logic
    def fly_circle(self):
        previous_time = time.time()
        while True:
            now = time.time()
            dt = now - previous_time
            if dt > self.interval:
                self.update_drone_position()
                self.circle.position = self.current_position
                u = self.distance_check_calculate_u(0.01, method='Circle')
                self.drone.send_rc(u)
                previous_time = now

    # Function that request an external module to update the drone's current position
    def update_drone_position(self):
        # Send request to external module
        self.need_new_position = True
        # Wait for a response
        while self.need_new_position:
            pass

    # Function that checks the distance sensors for values outside 'safe range' and calculates control command
    # method = 'Trapezoid', 'Proportional'
    def distance_check_calculate_u(self, bt_threshold, method='Trapezoid'):
        u = [0, 0, 0, 0]  # Assume the drone is to be stationary
        # Perform distance check and calculate velocities
        if not self.bluetooth.acceptL:
            if check_for_interval([self.bluetooth.current_package[0]], 0.001, bt_threshold):
                u = self.calculate_u(method)
        elif not self.bluetooth.acceptF:
            if check_for_interval([self.bluetooth.current_package[1]], 0.001, bt_threshold):
                u = self.calculate_u(method)
        elif not self.bluetooth.acceptR:
            if check_for_interval([self.bluetooth.current_package[2]], 0.001, bt_threshold):
                u = self.calculate_u(method)
        elif self.bluetooth.accept:
            if check_for_interval(self.bluetooth.current_package, 0.001, bt_threshold):
                u = self.calculate_u(method)
        else:
            # We do not have anything around and sensors are reading random values
            u = self.calculate_u(method)
        if method == 'Trapezoid':
            u[2], u[3] = 0, 0  # TODO: remove once Trapezoid is more stable
        return u

    # Function that calculates u
    def calculate_u(self, method='Trapezoid'):
        match method:
            case 'Trapezoid':
                u = self.trapezoid.calculate()
                # if self.bluetooth is not None:
                #     self.findObstacles()
                u = self.avoid(u)
                u[2], u[3] = 0, 0  # TODO: remove once Trapezoid is more stable
                return u
            case 'Proportional':
                u = self.proportional.getVel()
                # if self.bluetooth is not None:
                #     self.findObstacles()
                u = self.avoid(u)
                return u
            case 'Circle':
                return self.circle.calculate_angular_velocity()

    def point_from_relative_position(self):
        pos = self.current_position
        sens = self.bluetooth.current_package

        x = sens[1] * 100 * math.cos(math.radians(90 - pos[3]))
        y = sens[1] * 100 * math.sin(math.radians(90 - pos[3]))

        pos[0] += x
        pos[1] += y

        return pos

    def findObstacles(self):
        if self.bluetooth.acceptL or self.bluetooth.acceptR or self.bluetooth.acceptF:
            xs = self.current_position[0]
            ys = self.current_position[1]
            self.obstacleList = []
            if self.bluetooth.acceptL:
                tempyaw = self.current_position[3] + 270
                tempryaw = math.radians(tempyaw)
                dis = self.bluetooth.current_package[0]
                self.obstacleList.append([xs + math.sin(tempryaw) * dis * 100, ys + math.cos(tempryaw) * dis * 100])
            if self.bluetooth.acceptF:
                tempyaw = self.current_position[3]
                tempryaw = math.radians(tempyaw)
                dis = self.bluetooth.current_package[1]
                self.obstacleList.append([xs + math.sin(tempryaw) * dis * 100, ys + math.cos(tempryaw) * dis * 100])
            if self.bluetooth.acceptR:
                tempyaw = self.current_position[3] + 90
                tempryaw = math.radians(tempyaw)
                dis = self.bluetooth.current_package[2]
                self.obstacleList.append([xs + math.sin(tempryaw) * dis * 100, ys + math.cos(tempryaw) * dis * 100])

    def avoid(self, u, obstacle=None, method=2):
        match method:
            case 1:
                # speed * cos(alpha)
                x = self.MAX_SPEED * math.cos(math.radians(self.current_position[3]))
                y = self.MAX_SPEED * math.sin(math.radians(self.current_position[3]))
                magnitude = math.sqrt(obstacle[0] ** 2 + obstacle[1 ** 2])
                self.circle.radius = magnitude / 2
                self.circle.theta = math.radians(180)
                self.circle.speed = x
                control = self.circle.calculate_angular_velocity()
                control[0] = x
                control[1] = y
                return control
            case 2:
                # TRANSPOSE ALL THE THINGS
                panic = False
                ry = math.radians(self.current_position[3])
                for pos in self.obstacleList:
                    diffX = self.current_position[0] - pos[0]
                    diffY = self.current_position[1] - pos[1]
                    dis = math.sqrt(diffX ** 2 + diffY ** 2)
                    # print(f"Distance = {dis}")
                    rddAngle = math.atan2(-diffX, -diffY)
                    x = u[0] * math.cos(-ry) - u[1] * math.sin(-ry)
                    y = u[0] * math.sin(-ry) + u[1] * math.cos(-ry)
                    rtAngle = math.atan2(x, y)
                    if abs(rddAngle - rtAngle) >= .5 * math.pi:
                        # print("flying in opposite direction so it's safe")
                        pass
                    elif dis < 40:  # PANIC
                        # print("PANIC")
                        if panic:
                            # print("MULTIPLE PANIC")
                            factor = (60-math.sqrt(diffX**2 + diffY**2))/math.sqrt(diffX**2 + diffY**2)
                            u[0] += diffX*factor
                            u[1] += diffY*factor
                        else:
                            # print("FIRST PANIC")
                            panic = True
                            factor = (60-math.sqrt(diffX**2 + diffY**2))/math.sqrt(diffX**2 + diffY**2)
                            x = diffX * math.cos(ry) - diffY * math.sin(ry)
                            y = diffX * math.sin(ry) + diffY * math.cos(ry)
                            u[0], u[1] = x*factor, -y*factor
                            # print(u)
                    elif 40 <= dis <= 80 and not panic:  # curve around
                        # print("Curvy")
                        rAngle = math.atan2(diffX, diffY)
                        if rddAngle == rtAngle:
                            rAngle -= 0.05 * math.pi
                        x = u[0] * math.cos(rAngle - ry) - u[1] * math.sin(rAngle - ry)
                        u[0], u[1] = x, 0
                        x = u[0] * math.cos(-rAngle + ry) - u[1] * math.sin(-rAngle + ry)
                        y = u[0] * math.sin(-rAngle + ry) + u[1] * math.cos(-rAngle + ry)

                        scalar = 0.5
                        u[0], u[1] = x * scalar, y * scalar
                    else:
                        # print("It's fine")
                        pass
                return u
