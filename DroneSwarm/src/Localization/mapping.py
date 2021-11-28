# """custom path visualizer"""
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

    def endGraph(self):
        plt.show();
