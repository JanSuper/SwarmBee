from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import numpy as np

class CSI():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cs = CubicSpline(x, y)
        self.derivative = self.cs.derivative(1)

    def getPos(self, x):
        return self.cs(x)

    def getDer(self, x):
        return self.derivative(x)

    def debug(self):
        xs = np.arange((min(self.x) - 0.5), (max(self.x) + 0.5), 0.1)
        fig, ax = plt.subplots(figsize=(6.5, 4))
        ax.plot(self.x, self.y, 'o', label='data')
        ax.plot(xs, self.cs(xs), label="full")
        ax.plot(xs, self.getDer(xs), label="der")
        ax.legend(loc='lower left', ncol=2)
        plt.show()

def main():
    x = np.arange(10)
    y = np.sin(x)
    spline = CSI(x, y)
    spline.debug()

if __name__ == "__main__":
    main()
