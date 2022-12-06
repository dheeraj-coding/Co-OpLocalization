#!/usr/bin/env python

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

id = 1
suffix = 'test'

estFrame = pd.read_csv('iris{id}_{suffix}_est.csv'.format(id=id, suffix=suffix))
gtFrame = pd.read_csv('iris{id}_{suffix}_gt.csv'.format(id=id, suffix=suffix))

estPts = estFrame.to_numpy()
gtPts = gtFrame.to_numpy()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# ax = fig.add_subplot()

print(gtPts)

ax.plot(estPts[:, 0], estPts[:, 1], estPts[:, 2], marker='o')
ax.plot(gtPts[:, 0], gtPts[:, 1], gtPts[:, 2], marker='^')
# ax.plot(estPts[:, 2], marker='o')
# ax.plot(gtPts[:, 2], marker='^')
plt.show()

