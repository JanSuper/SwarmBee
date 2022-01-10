import math
from scipy.optimize import fsolve
import numpy as np
from sympy import Symbol, nsolve, sqrt

class Circle:

    # calculate the angular velocity given the speed and direction to spin in
    def calculate_angular_velocity(self):
        control = np.zeros(4, dtype=int)
        if self.clockwise:
            control[0] = self.speed
        else:
            control[0] = -1 * self.speed
        angular_velocity = -1 * round(control[0] / self.radius)
        control[3] = angular_velocity
        return control

    def __init__(self, lead_drone_position, drone_number=1, speed=20, clockwise=True, radius=100, n_drones=4):
        self.speed = speed
        self.clockwise = clockwise
        self.radius = radius  # radius of the orbit path
        self.n_drones = n_drones  # the number of drones
        self.theta = math.radians(360)/self.n_drones
        self.theta_i = (drone_number-1)*self.theta
        self.drone_number = drone_number
        self.last_drone_pos = [self.radius, 0, 0, 0]
        self.lead_drone_position = lead_drone_position

    def calculate_starting_positions(self):
        u = [0, 0, 0, 0]
        x1 = Symbol('x')
        y1 = Symbol('y')
        # print(self.last_drone_pos)
        # print(math.degrees(self.theta_i))
        # y1 - math.tan(self.theta_i) * x1
        x = round(self.radius*math.sin(self.theta_i))
        y = round(self.radius*math.cos(self.theta_i))
        # x, y = nsolve((x1 ** 2 - self.radius ** 2 - y1**2, y1 - x1*math.tan(self.theta_i)), (x1, y1),
        # (self.last_drone_pos[0], self.last_drone_pos[1]))

        self.last_drone_pos[0] = x
        self.last_drone_pos[1] = y
        self.drone_number += 1
        self.theta_i = (self.drone_number-1)*self.theta
        u[0], u[1] = x, y
        return u

    # TODO def face_center(self):
    # yaw drone i = round(180 mod (lead_drone yaw + degrees(theta_i)))

    # calculates the position of the center of orbit
    def orbit_center_position(self):
        u = self.lead_drone_position
        u[0] = u[0+self.radius]
        return u

    # math.radians(360) = degrees to radians
    # math.degrees(2*math.pi) = radians to degrees
    # math.sin/cos(radians)


if __name__ == "__main__":
    n_drones = 5
    circle = Circle([0, 0, 0, 0], n_drones=n_drones)
    for i in range(n_drones):
        print(f'control order is {circle.calculate_starting_positions()}')
