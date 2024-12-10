import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

import csv

# INIT
xs = []
ys = []
zs = []
x_raws = []
y_raws = []
z_raws = []

with open('../Lecture_07/data.csv', newline='') as csvfile:
    data = list(csv.reader(csvfile))


# Time [s], roll [deg], pitch [deg], yaw [deg], pos_x [m], pos_y [m], pos_z [m], pos_raw_x [m], pos_raw_y [m], pos_raw_z [m], position_initialized [b], position_raw_initialized [b]

# Insert raw data to array
for i in range(1, len(data)):
    # raw
    x_raws.append(float(data[i][7]))
    y_raws.append(float(data[i][8]))
    z_raws.append(float(data[i][9]))

# Insert fixed data to array
fixer_init = False
fixer = [0,0,0]
for i in range(1, len(data)):
    if (abs(float(data[i][4]) > 0.0)):
        if (not fixer_init):
            fixer[0] = float(data[i][4])
            fixer[1] = float(data[i][5])
            fixer[2] = float(data[i][6])
            fixer_init = True
        
        xs.append(float(data[i][4]) - fixer[0])
        ys.append(float(data[i][5]) - fixer[1])
        zs.append(float(data[i][6]) - fixer[2])

print(len(xs))

# Convert to np.array
xs = np.array(xs)
ys = np.array(ys)
zs = np.array(zs)
x_raws = np.array(x_raws)
y_raws = np.array(y_raws)
z_raws = np.array(z_raws)


# Remove outliers
# Remove any data points which diverges 10 or more from earlier data 
# for i in range(2, len(data)):
#     if 10 < abs(xs[i-1] - xs[i]):
#         xs[i] = xs[i-1]



# Init figure
fig = plt.figure()
ax = fig.add_subplot(projection='3d')


### Plot raw
# ax.plot(x_raws, y_raws, z_raws)

# # Equalize plot scale
# max_range = np.array([x_raws.max()-x_raws.min(), y_raws.max()-y_raws.min(), z_raws.max()-z_raws.min()]).max() / 2.0
# mid_x = (x_raws.max()+x_raws.min()) * 0.5
# mid_y = (y_raws.max()+y_raws.min()) * 0.5
# mid_z = (z_raws.max()+z_raws.min()) * 0.5
# ax.set_xlim(mid_x - max_range, mid_x + max_range)
# ax.set_ylim(mid_y - max_range, mid_y + max_range)
# ax.set_zlim(mid_z - max_range, mid_z + max_range)

# # Plot projection
# ax.plot(x_raws, y_raws, mid_z - max_range)
###


### plot fixed
ax.plot(xs, ys, zs)

# Equalize plot scale
max_range = np.array([xs.max()-xs.min(), ys.max()-ys.min(), zs.max()-zs.min()]).max() / 2.0
mid_x = (xs.max()+xs.min()) * 0.5
mid_y = (ys.max()+ys.min()) * 0.5
mid_z = (zs.max()+zs.min()) * 0.5
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

ax.plot(xs, ys, mid_z - max_range)
###



plt.show()