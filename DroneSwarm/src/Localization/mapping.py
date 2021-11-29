# """custom path visualizer"""
import math

import matplotlib.pyplot as plt
import numpy as np

GREEN_TEMP = 30
YELLOW_TEMP = 40

class Plot:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(projection='3d')
        self.ax.set_xlabel('X Label')
        self.ax.set_ylabel('Y Label')
        self.ax.set_zlabel('Z Label')

    def updateGraph(self,xs,ys,zs,yaw,temp):
        #TODO: integrate yaw
        if temp < GREEN_TEMP:
            self.ax.scatter(xs, ys, zs,marker='>',c="g")
        elif temp < YELLOW_TEMP:
            self.ax.scatter(xs, ys, zs, marker='>', c="y")
        else:
            self.ax.scatter(xs, ys, zs, marker='>', c="r")
        plt.pause(0.01)
        return

    def addObstacle(self,xs,ys,zs,dyaw,dir,dis):
        if dir == "l":
            dyaw += 90
        elif dir == "r":
            dyaw += 270

        print(dyaw)
        radyaw = dyaw/(180)*math.pi
        self.ax.scatter(xs+math.cos(radyaw)*dis, ys+math.sin(radyaw)*dis, zs, marker='s', c="purple")

    def endGraph(self):
        plt.show();
