import math


class Localiser:
    def __init__(self):
        self.camAngle = 90


    def calcPosWallX(self,x,y,z,r,ArUcoID):
        #Method to calculate the position of the drone if the ArUco marker is posted on a wall parallel to the X axis


        #TODO: implement different locations attached to ID
        Arucox = 0
        Arucoy = 200
        Arucoz = 170
        PosCof = 1

        xy = math.sqrt(x*x+y*y)
        #calc distance between drone and white dot
        dwls = math.sqrt(z*z + xy*xy - 2*z*xy*math.cos(r/180*math.pi))

        #calc horizontal component of line between drone and white dot
        dwlshz = math.cos((90-self.camAngle)/180*math.pi)*dwls

        #calc angle horizontal component and drone using r of aruco
        dwa = 90-r

        #calc distance between drone and wall
        dw = math.cos(dwa / 180 * math.pi) * dwlshz

        #calc difference in height between white dot and drone
        dotdronez = math.sin((90-self.camAngle)/180*math.pi)*dwls

        #calc difference in left/right between white dot and drone
        dotdroneLF = math.sin(dwa / 180 * math.pi) * dwlshz

        dx = Arucox + (x + dotdroneLF)*PosCof
        dy = Arucoy - dw
        dz = Arucoz + y + dotdronez
        dyaw = dwa
        return [dx,dy,dz,dyaw]


