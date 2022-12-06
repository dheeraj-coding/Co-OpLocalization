#!/usr/bin/env python

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

estFrame = pd.read_csv('iris1_dr4_est.csv')
gtFrame = pd.read_csv('iris1_dr4_gt.csv')

estPts = estFrame.to_numpy()
gtPts = gtFrame.to_numpy()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

print(gtPts)

ax.plot(estPts[:, 0], estPts[:, 1], estPts[:, 2], marker='o')
ax.plot(gtPts[:, 0], gtPts[:, 1], gtPts[:, 2], marker='^')

plt.show()

