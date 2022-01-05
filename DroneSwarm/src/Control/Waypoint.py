import queue
import matplotlib.pylab as plt
import numpy as np
import bezier

class Waypoints:

    def __init__(self):
        self.waypoints = queue.LifoQueue()

    def add_waypoint(self, waypoint):
        self.waypoints.put(waypoint)


def main():
    waypoints = Waypoints()
    waypoint = np.array([1, 8, 4, 3])
    waypoints.add_waypoint(waypoint)


if __name__ == "__main__":
    main()

