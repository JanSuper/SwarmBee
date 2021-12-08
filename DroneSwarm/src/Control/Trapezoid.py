import numpy as np

MAX_SPEED = 20  # cm/s
ACCELERATION_RATE = 2  # cm/s^2


# noinspection PyUnresolvedReferences

# Function to check if all the values of list1 are greater than val
# If all values are greater than val, return True else return False
def check_for_greater(list1, val):
    # traverse in the list
    i_s = [True, True, True, True]
    i = 0
    for x in list1:
        # compare with all the
        # values with value
        if x >= val:
            i_s[i] = False
        i += 1
    return i_s


class Trapezoid:
    def __init__(self, px=1, py=1, pz=1, pyaw=1):
        p = np.array([-px, -py, -pz, -pyaw])
        self.trapezoid = p
        # array values are [x, y, z, yaw]
        self.position = np.zeros(4)  # current position as origin
        self.target = np.zeros(4)  # target position as origin
        self.velocity = np.zeros(4, dtype=int)  # initial velocities as 0
        self.distance = np.zeros(4)  # current distance to target
        self.is_reached = [True, True, True, True]
        self.reached = False

    def set_position(self, position):
        if position.shape == self.position.shape:
            self.position = position
        else:
            print("Invalid position")

    def update_position_estimate(self, dt, x=0.0, y=0.0, z=0.0, yaw=0):
        temp = np.array([x, y, z, yaw])
        self.position[:] = self.position[:] + temp[:] * dt
        self.position[3] %= 180  # +-180 in degrees
        return self.position

    def update_position_true(self, dt, x=0, y=0.0, z=0.0, yaw=0):
        temp = np.array([x, y, z, yaw])
        self.position[1:3] = self.position[1:3] + (temp[1:3] * dt)
        self.position[3] = temp[3]
        return self.position

    def set_target(self, target):
        if target.shape == self.target.shape:
            if target.dtype != np.int:
                print("Target position must be an integer")
            else:
                self.target[:] = target[:]
                self.distance[:] = self.target[:] - self.position[:]
                # print(self.distance.tolist())
                self.is_reached = check_for_greater(self.distance.tolist(), 10)
                if any(self.is_reached):
                    self.reached = False
        else:
            print("Invalid target position")

    # TODO: add docs
    def calculate(self):
        for i in range(4):
            if abs((self.target[i] - self.position[i])) > abs((2 * self.distance[i] / 3)):
                if abs(self.velocity[i]) < MAX_SPEED:
                    if self.target[i] - self.position[i] > 0:
                        self.velocity[i] += ACCELERATION_RATE
                    elif self.target[i] - self.position[i] < 0:
                        self.velocity[i] -= ACCELERATION_RATE
                else:
                    if self.target[i] - self.position[i] > 0:
                        self.velocity[i] = MAX_SPEED
                    elif self.target[i] - self.position[i] < 0:
                        self.velocity[i] = -MAX_SPEED
            elif abs((self.target[i] - self.position[i])) <= abs((self.distance[i] / 3)):
                if abs(self.position[i] - self.target[i]) > 10:
                    if abs(self.velocity[i]) > 11:
                        if self.target[i] - self.position[i] > 0:
                            self.velocity[i] -= ACCELERATION_RATE
                        elif self.target[i] - self.position[i] < 0:
                            self.velocity[i] += ACCELERATION_RATE
                    else:
                        if self.target[i] - self.position[i] > 0:
                            self.velocity[i] = 11
                        elif self.target[i] - self.position[i] < 0:
                            self.velocity[i] = -11
                else:
                    self.is_reached[i] = True
                    self.velocity[i] = 0
        # print(self.is_reached)
        if all(self.is_reached):
            self.reached = True
            print("Reached Target: %s" % self.target)
            # print('\n Enter a new target position! Current position:%s' % self.position)

        np.clip(self.velocity, -MAX_SPEED, MAX_SPEED)
        velocity = self.velocity.tolist()
        return velocity
