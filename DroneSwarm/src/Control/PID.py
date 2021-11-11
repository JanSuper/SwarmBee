import math
import matplotlib.pyplot as plt
import numpy as np

plt.style.use('fivethirtyeight')

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
pos = []
tar = []
con = []
tim = []


# noinspection PyUnresolvedReferences
class PID:
    def __init__(self, px=1, py=1, pz=1, pyaw=0.5):
        p = np.array([-px, -py, -pz, -pyaw])
        for i in range(4):
            t[i, :] = t[i, :] * p[i]
        self.pid = t  # parameter values
        print("Co-efficient matrix: \n", self.pid)
        # array values are [x, y, z, yaw]
        self.position = np.zeros(4)  # current position as origin
        self.target = np.zeros(4)  # target position as origin
        self.velocity = np.zeros(4)  # initial velocities as 0
        self.error = np.zeros((4, 4))
        # [[error_x,   integral_error_x,   derivative_error_x,   previous_error_x]
        #  [error_y,   integral_error_y,   derivative_error_y,   previous_error_y]
        #  [error_z,   integral_error_z    derivative_error_z,   previous_error_z]
        #  [error_yaw, integral_error_yaw, derivative_error_yaw, previous_error_yaw]
        # 4x4

    def update_position_estimate(self, dt, x=0, y=0, z=0, yaw=0):
        temp = np.array([x, y, z, yaw])
        self.position[:] = self.position[:] + temp[:] * dt
        self.position[3] %= 180  # +-180 in degrees
        return self.position

    def update_position_true(self, dt, x=0.0, y=0.0, z=0.0, yaw=0.0):
        temp = np.array([x, y, z, yaw])
        self.position[1:3] = self.position[1:3] + (temp[1:3] * dt)
        self.position[3] += temp[3]
        self.position[3] = 180  # +-180 in degrees
        return self.position

    def set_position(self, position):
        if position.shape == self.position.shape:
            self.position = position
        else:
            print("Invalid position")

    def set_target(self, target):
        if target.shape == self.target.shape:
            if target.dtype != np.int:
                print("Target position must be an integer")
            else:
                self.target[:] = target[:]
                self.reset()
        else:
            print("Invalid target position")

    # TODO: add multi-(threading)processing
    def calculate(self, dt):
        if not np.array_equal(self.target, self.position):
            self.error[:, 0] = self.target[:] - self.position[:]   # P controller
            self.error[:, 1] += self.error[:, 0] * dt  # I controller
            self.error[:, 2] = (self.error[:, 0] - self.error[:, 3]) / dt  # D controller
            self.windup()
            u = self.control_signal()
            self.error[:, 3] = self.error[:, 0]  # set previous error to current
            return u
        else:
            print("Reached the target position")

    # noinspection PyUnresolvedReferences
    def control_signal(self):
        u = np.zeros(4, dtype=int)
        u[:] = self.pid[:, 0] * self.error[:, 0] + self.pid[:, 1] * self.error[:, 1] + self.pid[:, 2] * self.error[:, 2]
        u = np.clip(u, -50, 50)
        u = u.tolist()
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

    def plotter(self, elapsed_time, control, cls=False):
        pos.append(self.position[3])
        tar.append(self.target[3])
        con.append(control[3])
        tim.append(elapsed_time)
        plt.plot(tim, pos, label='Position')
        plt.plot(tim, tar, label='Target')
        # plt.plot(tim, con, label='Control')
        plt.xlim([0, 15])
        plt.xlabel('Time (s)')
        plt.ylabel('cm, cm/s')
        plt.legend(loc='upper left')
        plt.show()
        if cls:
            plt.close()
