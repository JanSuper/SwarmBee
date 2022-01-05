import numpy as np


class Circle:

    # calculate the angular velocity given the speed and direction to spin in
    def calculate_angular_velocity(self):
        control = np.zeros(4, dtype=int)
        if self.clockwise:
            control[0] = -1 * self.speed
        else:
            control[0] = self.speed
        angular_velocity = -1 * round(control[0] / self.radius)
        control[3] = angular_velocity
        return control

    def __init__(self, speed=20, clockwise=True, radius=100):
        self.speed = speed
        self.clockwise = clockwise
        self.radius = radius
