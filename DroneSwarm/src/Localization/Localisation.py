import math
import numpy as np


class Localiser:
    def __init__(self):
        self.camAngle = 90


    def calcPosFloor(self,x,y,z,r,offset):
        arucoX = offset[0]
        arucoY = 200 - offset[1]

        vector_1 = [0, 1]
        vector_2 = [x, y]

        unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
        unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
        dot_product = np.dot(unit_vector_1, unit_vector_2)
        angle = np.arccos(dot_product)

        #print(angle)

        vlength = np.sqrt(math.pow(x,2) + math.pow(y,2))

        radYaw = math.radians(r)

        # print("------")
        # print(vector_2)
        # print(angle)
        # print(radYaw)

        realDistance = [x * math.cos(-radYaw) - y * math.sin(-radYaw),x * math.sin(-radYaw) + y *math.cos(-radYaw)]

        # print(realDistance)


        realPos = [-realDistance[0] + arucoX, realDistance[1] + arucoY]

        return realPos

    # def calcPosWallX(self,x,y,z,r,ArUcoID):
    #     #Method to calculate the position of the drone if the ArUco marker is posted on a wall parallel to the X axis
    #
    #     # for markers 0 to 3: value at index 0 is dist. from 0,0 point on x-axis
    #     # for markers 4 to 7: value at index 0 is 0 for simplification of calculations
    #     # for all markers:    value at index 1 is height of the marker center
    #     # for markers 4 to 7: value at index 2 is distance to the corner of the walls
    #     offsetsTemp = [[0, 170], [0, 100], [154, 100], [154, 170], [0, 170, 200], [0, 100, 200],
    #                    [0, 100, 100], [0, 170, 100]]
    #     corner_loc = -101  # corner location on x axis = -101
    #
    #     # for markers 4 ... 7:
    #     # drone x = corner_loc + dy
    #     # drone y = -dx + the marker's center's distance from the corner
    #     #
    #
    #     #TODO: implement different locations attached to ID
    #     Arucox = 0
    #     Arucoy = offsetsTemp[ArUcoID][0]
    #     Arucoz = offsetsTemp[ArUcoID][1]
    #     PosCof = 1
    #
    #     xy = math.sqrt(x*x+y*y)
    #     #calc distance between drone and white dot
    #     dwls = math.sqrt(z*z + xy*xy - 2*z*xy*math.cos(r/180*math.pi))
    #
    #     #calc horizontal component of line between drone and white dot
    #     dwlshz = math.cos((90-self.camAngle)/180*math.pi)*dwls
    #
    #     #calc angle horizontal component and drone using r of aruco
    #     dwa = 90-r
    #
    #     #calc distance between drone and wall
    #     dw = math.cos(dwa / 180 * math.pi) * dwlshz
    #
    #     #calc difference in height between white dot and drone
    #     dotdronez = math.sin((90-self.camAngle)/180*math.pi)*dwls
    #
    #     #calc difference in left/right between white dot and drone
    #     dotdroneLF = math.sin(dwa / 180 * math.pi) * dwlshz
    #
    #     #dx = Arucox + (-x + dotdroneLF)*PosCof
    #     #dy = Arucoy - dw
    #     dx = Arucoy + dw*PosCof
    #     dy = Arucox + (-x + dotdroneLF)*PosCof
    #     dz = Arucoz + y + dotdronez
    #     dyaw = dwa
    #
    #     if ArUcoID in range(4,8):
    #         dx = corner_loc + dy
    #         dy = offsetsTemp[ArUcoID][2] - dx
    #
    #     return [dx,dy,dz,dyaw]



