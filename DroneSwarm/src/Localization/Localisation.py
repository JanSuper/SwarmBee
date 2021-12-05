import math


class Localiser:
    def __init__(self):
        self.camAngle = 0


    def calcPosWall(self,x,y,z,r,ArUcoID):
        #TODO: implement different locations attached to ID

        xy = math.sqrt(x*x+y*y)
        #calc distance from wall of camera sight using cosine rule
        dwls = math.sqrt(z*z + xy*xy - 2*z*xy*math.cos(r/180*math.pi))

        #use sinerule to calc angle between line of sight and wall
        #might be redundant
        sineRule = dwls/math.sin(r/180*math.pi)
        whiteDotAngle = math.asin(dwls/sineRule)

        dw = math.cos(self.camAngle/180*math.pi)*dwls

        dx = 0
        dy = 0
        dz = 0
        dyaw = 0
        return [dx,dy,dz,dyaw]


