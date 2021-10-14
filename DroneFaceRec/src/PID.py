import time

class PID:
    def __init__(self, drone): #
        self.pid = [1.0, 0.5, 0.3] #PID VALUES
        self.drone = drone
        self.currentX = 0
        self.desiredX = 0
        self.currentY = 0
        self.desiredY = 0
        self.currentZ = 0
        self.desiredZ = 0
        self.currentYaw = 0
        self.desiredYaw = 0
        self.Xvel = 0
        self.Yvel = 0
        self.Zvel = 0
        self.YawVel = 0
        self.XI = 0
        self.YI = 0
        self.ZI = 0
        self.YawI = 0
        self.XprevE = 0
        self.YprevE = 0
        self.ZprevE = 0
        self.YawprevE = 0
        self.previousTime = time.time()
        self.currentTime = 0
        self.deltaTime = 0

    def setDesiredPos(self, X, Y, Z, Yaw):
        self.desiredX = X
        self.desiredY = Y
        self.desiredZ = Z
        self.desiredYaw = Yaw
        self.XprevE, self.YprevE, self.ZprevE, self.YawprevE = 0
        self.XI, self.YI, self.ZI, self.YawI = 0

    def calcVel(self):
        self.currentTime = time.time()
        self.deltaTime = self.currentTime - self.previousTime
        self.increaseI()
        self.Xvel = self.calcPIDVel(self, self.desiredX - self.currentX, self.XI, self.XPrevE)
        self.XprevE = self.desiredX - self.currentX
        self.Yvel = self.calcPIDVel(self, self.desiredY - self.currentY, self.YI, self.YPrevE)
        self.YprevE = self.desiredY - self.currentY
        self.Zvel = self.calcPIDVel(self, self.desiredZ - self.currentZ, self.ZI, self.ZPrevE)
        self.ZprevE = self.desiredZ - self.currentZ
        self.Yawvel = self.calcPIDVel(self, self.desiredYaw - self.currentYaw, self.YawI, self.YawPrevE)
        self.YawprevE = self.desiredYaw - self.currentYaw
        self.previousTime = self.currentTime

    def calcPIDVel(self, error, I, PrevE):
        D = (error-PrevE)/self.deltaTime
        u = self.pid[0] * error + self.pid[1] * I + self.pid[2] * D
        return u

    def increaseI(self):
        self.XI += self.XI + (self.desiredX - self.currentX) * self.deltaTime
        self.YI += self.YI + (self.desiredY - self.currentY) * self.deltaTime
        self.ZI += self.ZI + (self.desiredZ - self.currentZ) * self.deltaTime
        self.YawI += self.YawI + (self.desiredYaw - self.currentYaw) * self.deltaTime

