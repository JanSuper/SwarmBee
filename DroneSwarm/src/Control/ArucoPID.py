import math


class APID():
    def __init__(self,x,y,z,yaw):
        self.x = 0
        self.y = 0
        self.z = 0
        self.yaw = 0

        self.desx = x
        self.desy = y
        self.desz = z
        self.desyaw = yaw


    def realUpdate(self,x,y,z,yaw):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.ryaw = math.radians(yaw)

    def setDes(self,x,y,z,yaw):
        self.desx = x
        self.desy = y
        self.desz = z
        self.desyaw = yaw
        self.ryaw = math.radians(yaw)

    def getVel(self):
        pre = [self.desx - self.x, self.desy - self.y]
        trans = [pre[0] * math.cos(-self.ryaw) - pre[1] * math.sin(-self.ryaw), pre[0] * math.sin(-self.ryaw)
                 + pre[1] * math.cos(-self.ryaw)]
        trans.append(self.desz-self.z)
        trans.append(self.desyaw - self.yaw)

        return trans

