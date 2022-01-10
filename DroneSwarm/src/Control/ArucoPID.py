import math
import numpy as np


class APID():

    def __init__(self, des, ryaw=0):
        self.x = 0
        self.y = 0
        self.z = 0
        self.yaw = 0

        self.desx = des[0]
        self.desy = des[1]
        self.desz = des[2]
        self.desyaw = des[3]

        self.ryaw = ryaw

    def realUpdate(self, x, y, z, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.ryaw = math.radians(yaw)

    def realUpdate(self, pos):
        self.x = pos[0]
        self.y = pos[1]
        self.z = pos[2]
        self.yaw = pos[3]
        self.ryaw = math.radians(pos[3])

    def setDes(self, x, y, z, yaw):
        self.desx = x
        self.desy = y
        self.desz = z
        self.desyaw = yaw

    def setDes(self, desPos):
        self.desx = desPos[0]
        self.desy = desPos[1]
        self.desz = desPos[2]
        self.desyaw = desPos[3]

    def getVel(self):
        print([self.desx, self.desy, self.desz, self.desyaw])
        print([self.x, self.y, self.z, self.yaw])
        pre = [self.desx - self.x, self.desy - self.y]
        trans = [pre[0] * math.cos(self.ryaw) - pre[1] * math.sin(self.ryaw), pre[0] * math.sin(self.ryaw)
                 + pre[1] * math.cos(self.ryaw), self.desz - self.z, -(self.desyaw - self.yaw)]
        print(trans)
        trans = list(np.around(np.array(trans), decimals=1))
        print(trans)
        return trans
