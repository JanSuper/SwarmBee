# """custom path visualizer"""
import matplotlib.pyplot as plt
import numpy as np

class Plot:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(projection='3d')
        self.ax.set_xlabel('X Label')
        self.ax.set_ylabel('Y Label')
        self.ax.set_zlabel('Z Label')

    def updateGraph(self,xs,ys,zs,yaw,temp):
        #TODO: integrate temprature and yaw
        self.ax.scatter(xs, ys, zs,marker='>')
        plt.pause(0.01)
        return

    def endGraph(self):
        plt.show();
