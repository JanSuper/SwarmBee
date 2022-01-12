import math
from scipy.optimize import fsolve
import numpy as np
from sympy import Symbol, nsolve, sqrt


class Circle:

    # calculate the angular velocity given the speed and direction to spin in
    def calculate_angular_velocity(self):
        control = [0, 0, 0, 0]
        if self.clockwise:
            control[0] = -1 * self.speed
        else:
            control[0] = self.speed
        print(math.degrees(control[1] / self.radius))
        angular_velocity = -1 * round(math.degrees(control[1] / self.radius))
        control[3] = angular_velocity
        return control

    def __init__(self, drone_number=1, speed=50, theta=360, clockwise=False, facing_center=False, in_position = False, radius=100, n_drones=4, center=None,
                 position=None):
        if position is None:
            position = [0, 0, 0, 0]
        if center is None:
            center = [0, 0, 0, 0]
        self.speed = speed
        self.clockwise = clockwise
        self.radius = radius  # radius of the orbit path
        self.n_drones = n_drones  # the number of drones
        self.theta = math.radians(theta)/self.n_drones
        self.drone_number = drone_number
        self.position = position
        self.in_position = in_position
        self.center_of_orbit = center
        self.facing_center = facing_center

    def calculate_starting_positions(self):
        starting_pos = [0, 0, 0, 0]
        x = self.radius*math.cos((self.drone_number-1)*self.theta)
        y = self.radius*math.sin((self.drone_number-1)*self.theta)
        starting_pos[0], starting_pos[1] = x, y
        return starting_pos

    # TODO def face_center(self):
    # def face_center(self):
    #     if not self.in_position:
    #         print('We are not in position (apparently)')
    #         return [0, 0, 0, 0]
    #     else:
    #
        # yaw_drone_i = 180 mod (lead_drone yaw + degrees(theta_i))

    # math.radians(360) = degrees to radians
    # math.degrees(2*math.pi) = radians to degrees
    # math.sin/cos(radians)

    # might not need it but calculates distance travelled given a sector's radius and radians.
    def arc(self, radians=True):
        if self.diameter:
            r = self.radius/2
        else:
            r = self.radius
        if radians:
            theta = self.angle
        else:
            theta = math.radians(self.angle)
        s = r*theta
        return s


if __name__ == "__main__":
    n = 5
    d = 1
    circle = Circle(drone_number=d, n_drones=n)
    print(f'circle speed is {circle.speed}')
    for i in range(n):
        print(f'Starting position is {circle.calculate_starting_positions()}')
        print(f'control order(for angular vel) is {circle.calculate_angular_velocity()}')
        circle.drone_number += 1
