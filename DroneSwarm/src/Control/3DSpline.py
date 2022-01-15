import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import random

# set seed to reproducible
random.seed(1)
data_size = 51
max_value_range = 132651
x = np.array([random.random()*max_value_range for p in range(0,data_size)])
y = np.array([random.random()*max_value_range for p in range(0,data_size)])
z = 2*x*x*x + np.sqrt(y)*y + random.random()
fig = plt.figure(figsize=(10,6))
ax = axes3d.Axes3D(fig)
ax.scatter3D(x,y,z, c='r')