import math
import numpy as np


class APID():

    def __init__(self, des, ryaw=0):
        print(f"(Proportional) New target = {des}")

        self.x = 0
        self.y = 0
        self.z = 0
        self.yaw = 0

        self.desx = des[0]
        self.desy = des[1]
        self.desz = des[2]
        self.desyaw = des[3]

        self.ryaw = ryaw

        self.reachedTarget = False
        self.obstacleFound = False
        self.obstacleList = [[0, 0], [0, 0], [0, 0]]

        self.acceptR = False
        self.acceptF = False
        self.acceptL = False

        self.current_package = [0,0,0]

    def realUpdate(self, x, y, z, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.ryaw = math.radians(yaw)
        print(f"(Proportional) New position = [{x}, {y}, {z}, {yaw}]")
        self.areWeThereYet()

    def realUpdate(self, pos):
        self.x = pos[0]
        self.y = pos[1]
        self.z = pos[2]
        self.yaw = pos[3]
        self.ryaw = math.radians(pos[3])
        print(f"(Proportional) New position = {pos}")
        self.areWeThereYet()

    def setDes(self, x, y, z, yaw):
        self.desx = x
        self.desy = y
        self.desz = z
        self.desyaw = yaw
        print(f"(Proportional) New target = [{x}, {y}, {z}, {yaw}]")
        self.reachedTarget = False
        self.areWeThereYet()

    def setDes(self, desPos):
        self.desx = desPos[0]
        self.desy = desPos[1]
        self.desz = desPos[2]
        self.desyaw = desPos[3]
        print(f"(Proportional) New target = {desPos}")
        self.reachedTarget = False
        self.areWeThereYet()

    def areWeThereYet(self):
        diffX = self.desx - self.x
        diffY = self.desy - self.y
        diffZ = self.desz - self.z

        MAX_DISTANCE = math.sqrt(300)

        if not self.reachedTarget:
            self.reachedTarget = math.sqrt(diffX ** 2 + diffY ** 2 + diffZ ** 2) <= MAX_DISTANCE

    def setObstacle(self, list):
        self.obstacleFound = True
        self.obstacleList = list

    def getVel(self):
        if self.reachedTarget:
            return [0, 0, 0, 0]
        self.findObstacles()
        # print([self.desx, self.desy, self.desz, self.desyaw])
        # print([self.x, self.y, self.z, self.yaw])
        pre = [self.desx - self.x, self.desy - self.y]
        x = pre[0] * math.cos(self.ryaw) - pre[1] * math.sin(self.ryaw)
        y = pre[0] * math.sin(self.ryaw) + pre[1] * math.cos(self.ryaw)
        if min(abs(self.desyaw - self.yaw), (360 - abs(self.desyaw - self.yaw))) == abs(self.desyaw - self.yaw):
            yawVel = ((self.desyaw - self.yaw)) / math.sqrt(x ** 2 + y ** 2)
        else:
            yawVel = -(360 - (self.desyaw - self.yaw)) / math.sqrt(x ** 2 + y ** 2)
        trans = [x, y, self.desz - self.z, yawVel]
        # trans = [pre[0] * math.cos(self.ryaw) - pre[1] * math.sin(self.ryaw), pre[0] * math.sin(self.ryaw)
        #          + pre[1] * math.cos(self.ryaw), self.desz - self.z, -(self.desyaw - self.yaw)]
        # print(trans)
        trans = list(np.around(np.array(trans), decimals=1))
        # print(trans)
        # if self.obstacleFound:
        #    trans = self.avoidObstacles(trans)
        p_value = 0.5
        trans[0] *= p_value
        trans[1] *= p_value
        trans[2] *= p_value
        return trans

    def avoidObstacles(self, trans):
        # TRANSPOSE ALL THE THINGS
        print("There are obstacles")
        panic = False
        speed = math.sqrt(trans[0] ** 2 + trans[1] ** 2)

        for pos in self.obstacleList:
            print("new pos")
            diffX = self.x - pos[0]
            diffY = self.y - pos[1]
            dis = math.sqrt(diffX ** 2 + diffY ** 2)
            rddAngle = math.atan2(-diffX, -diffY)
            x = trans[0] * math.cos(-self.ryaw) - trans[1] * math.sin(-self.ryaw)
            y = trans[0] * math.sin(-self.ryaw) + trans[1] * math.cos(-self.ryaw)
            rtAngle = math.atan2(x, y)
            if dis < 20:  # PANIC
                print("PANIC")
                if panic:
                    print("MULTIPLE PANIC")
                    trans[0] += diffX
                    trans[1] += diffY
                else:
                    print("FIRST PANIC")
                    panic = True
                    x = diffX * math.cos(self.ryaw) - diffY * math.sin(self.ryaw)
                    y = diffX * math.sin(self.ryaw) + diffY * math.cos(self.ryaw)
                    trans = [x, y, trans[2], trans[3]]
                    print(trans)
            elif (abs(rddAngle - rtAngle)) >= (.5 * math.pi):
                print("flying in opposite direction so it's safe")
                pass
            elif 20 <= dis <= 60 and not panic:  # curve around
                print("Curvy")
                rAngle = math.atan2(diffX, diffY)
                if rddAngle == rtAngle:
                    rAngle -= 0.05 * math.pi
                x = trans[0] * math.cos(rAngle - self.ryaw) - trans[1] * math.sin(rAngle - self.ryaw)
                trans = [x, 0, trans[2], trans[3]]
                x = trans[0] * math.cos(-rAngle + self.ryaw) - trans[1] * math.sin(-rAngle + self.ryaw)
                y = trans[0] * math.sin(-rAngle + self.ryaw) + trans[1] * math.cos(-rAngle + self.ryaw)
                trans = [x, y, trans[2], math.degrees(rddAngle - self.ryaw)]
            else:
                print("It's fine")
                pass
        return trans

    def findObstacles(self):
        if self.acceptL or self.acceptR or self.acceptF:
            xs = self.x
            ys = self.y
            obstacleList = []
            if self.acceptL:
                tempyaw = self.yaw + 270
                tempryaw = math.radians(tempyaw)
                dis = self.current_package[0]
                obstacle = [xs + math.sin(tempryaw) * dis * 100, ys + math.cos(tempryaw) * dis * 100]
                print(obstacle)
                obstacleList.append(obstacle)
            if self.acceptF:
                tempyaw = self.yaw
                tempryaw = math.radians(tempyaw)
                dis = self.current_package[1]
                obstacle = [xs + math.sin(tempryaw) * dis * 100, ys + math.cos(tempryaw) * dis * 100]
                print(obstacle)
                obstacleList.append(obstacle)
            if self.acceptR:
                tempyaw = self.yaw + 90
                tempryaw = math.radians(tempyaw)
                dis = self.current_package[2]
                obstacle = [xs + math.sin(tempryaw) * dis * 100, ys + math.cos(tempryaw) * dis * 100]
                print(obstacle)
                obstacleList.append(obstacle)
            self.setObstacle(obstacleList)


def main():
    pid = APID([100, 0, 0, 0])
    #pid.setObstacle([[0,20]])
    pid.realUpdate([0, 0, 0, 90])
    print(pid.getVel())


if __name__ == "__main__":
    main()
