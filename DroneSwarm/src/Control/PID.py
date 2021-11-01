import math
import time

import numpy as np

# PID in standard form
#                  [1, 1/Txi, Txd]
#                  [1, 1/Tyi, Tyd]
#                  [1, 1/Tzi, Tzd]
#                  [1, 1/Tyawi, Tyawd]
#               4x3
t = np.array([(1, 1 / math.inf, 0),
              (1, 1 / math.inf, 0),
              (1, 1 / math.inf, 0),
              (1, 1 / math.inf, 0)], dtype=float)


class PID:
    def __init__(self, px=1, py=1, pz=1, pyaw=1):
        p = np.array([-px, -py, -pz, -pyaw])
        for i in range(4):
            t[i, :] = t[i, :] * p[i]
        self.pid = t  # parameter values
        print("Co-efficient matrix: ", self.pid)
        # array values are [x, y, z, yaw]
        self.position = np.zeros(4, dtype=int)  # current position as origin
        self.target = np.zeros(4, dtype=int)  # target position as origin
        self.velocity = np.zeros(4, dtype=int)  # initial velocities as 0
        self.error = np.zeros((4, 4))
        # [[error_x,   integral_error_x,   derivative_error_x,   previous_error_x]
        #  [error_y,   integral_error_y,   derivative_error_y,   previous_error_y]
        #  [error_z,   integral_error_z    derivative_error_z,   previous_error_z]
        #  [error_yaw, integral_error_yaw, derivative_error_yaw, previous_error_yaw]
        # 4x4
        self.previousTime = time.time()
        self.currentTime = 0
        self.deltaTime = 0

    def update_position(self, x=0, y=0, z=0, yaw=0):
        self.position[0] += x
        self.position[1] += y
        self.position[2] += z
        self.position[3] += yaw

    def set_position(self, position):
        if position.shape == self.position.shape:
            self.position = position
        else:
            print("Invalid position")

    def set_target(self, target):
        if target.shape == self.target.shape:
            if target.dtype != int:
                print("Target position must be an integer")
            else:
                self.target = target
                self.reset()
        else:
            print("Invalid target position")

    # TODO: add multi-(threading)processing
    def calculate(self, dt):
        if self.position.all() != self.target.all():
            self.error[:, 0] = self.position[:] - self.target[:]  # P controller
            self.error[:, 1] += self.error[:, 0] * dt  # I controller
            self.error[:, 2] = (self.error[:, 0] - self.error[:, 3]) / dt  # D controller
            self.windup()
            u = self.control_signal()
            self.error[:, 3] = self.error[:, 0]  # set previous error to current
            return u
        else:
            print("Reached the target position")

    def control_signal(self):
        u = np.zeros(4, dtype=int)
        u[:] = self.pid[:, 0] * self.error[:, 0] + self.pid[:, 1] * self.error[:, 1] + self.pid[:, 2] * self.error[:, 2]
        u = np.clip(u, -100, 100)
        # for i in range(4):
        #     if abs(u[i]) < 10:
        #         u[i] = 0
        return u

    def windup(self):
        for x in range(4):
            if abs(self.error[x, 0]) <= 1:
                self.error[x, 1] = 0  # reset the integral error
            if abs(self.error[x, 1]) >= 10:
                self.error[x, 1] = 0  # disable if error over-accumulates

    def reset(self):
        self.error[:, 1:4:2] = 0
