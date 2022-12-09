#!/usr/bin/env python

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys

id = sys.argv[1]
id = int(id)
suffix = sys.argv[2]

estFrame = pd.read_csv('iris{id}_{suffix}_est.csv'.format(id=id, suffix=suffix))
gtFrame = pd.read_csv('iris{id}_{suffix}_gt.csv'.format(id=id, suffix=suffix))

estPts = estFrame.to_numpy()
gtPts = gtFrame.to_numpy()

diff = estPts - gtPts
sq = np.square(diff)
asum = np.sum(sq)/len(sq)
rmse = np.sqrt(asum)
print(rmse)

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# # ax = fig.add_subplot()

# ax.plot(estPts[:, 0], estPts[:, 1], estPts[:, 2], marker='o')
# ax.plot(gtPts[:, 0], gtPts[:, 1], gtPts[:, 2], marker='^')
# # ax.plot(estPts[:, 2], marker='o')
# # ax.plot(gtPts[:, 2], marker='^')
# plt.show()

figs, axs = plt.subplots(2, 2)

figs.set_size_inches(18, 15)
axs[0, 0].plot(estPts[:, 0], estPts[:, 1], marker='o')
axs[0, 0].plot(gtPts[:, 0], gtPts[:, 1], marker='^')
axs[0, 0].set_title('XY Path', fontsize=10)
axs[0, 0].set_xlabel('x-axis')
axs[0, 0].set_ylabel('y-axis')

axs[0, 1].plot(estPts[:, 1], estPts[:, 2], marker='o')
axs[0, 1].plot(gtPts[:, 1], gtPts[:, 2], marker='^')
axs[0, 1].set_title('YZ Path', fontsize=10)
axs[0, 1].set_xlabel('x-axis')
axs[0, 1].set_ylabel('y-axis')

axs[1, 0].plot(estPts[:, 0], estPts[:, 2], marker='o')
axs[1, 0].plot(gtPts[:, 0], estPts[:, 2], marker='^')
axs[1, 0].set_title('XZ Path', fontsize=10)
axs[1, 0].set_xlabel('x-axis')
axs[1, 0].set_ylabel('y-axis')

sq = np.sum(sq, axis=1)
dist = np.sqrt(sq)
axs[1, 1].plot(dist)
axs[1, 1].set_title('Error vs Time', fontsize=10)
# axs[1, 1].text(0.5, 0.5, 'Avg RMSE: {val}'.format(val=round(rmse, 4)), horizontalalignment='left', verticalalignment='top')
axs[1, 1].set_xlabel('time')
axs[1, 1].set_ylabel('error')

# plt.subplot_tool()
plt.figtext(0.5, 0.01, 'Avg RMSE: {val}'.format(val=round(rmse, 4)), horizontalalignment='center', fontsize=15)
plt.subplots_adjust(hspace=0.30)

plt.savefig('iris{id}_{suffix}.png'.format(id=id, suffix=suffix))

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot(estPts[:, 0], estPts[:, 1], estPts[:, 2], marker='o')
ax.plot(gtPts[:, 0], gtPts[:, 1], gtPts[:, 2], marker='^')

fig.savefig('iris{id}_3D{suffix}.png'.format(id=id, suffix=suffix))

plt.show()


