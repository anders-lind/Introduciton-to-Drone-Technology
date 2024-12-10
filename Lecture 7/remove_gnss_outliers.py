import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import math

import csv



class Remove_GNSS_Outliers:

    def __init__(self, data_path = "data.csv"):
        # Parameters
        self.data_path = data_path

        # INIT
        self.times = []
        self.xs = []
        self.ys = []
        self.zs = []
        self.x_filtered = []
        self.y_filtered = []
        self.z_filtered = []


    def load_data(self):
        with open(self.data_path, newline='') as csvfile:
            self.data = list(csv.reader(csvfile))
            self.offset_data_to_zero()


    # Time [s], roll [deg], pitch [deg], yaw [deg], pos_x [m], pos_y [m], pos_z [m], pos_raw_x [m], pos_raw_y [m], pos_raw_z [m], position_initialized [b], position_raw_initialized [b]


    def offset_data_to_zero(self):
        # Insert fixed data to array
        fixer_init = False
        fixer = [0,0,0]
        for i in range(1, len(self.data)):
            if (abs(float(self.data[i][4]) > 0.0)):
                if (not fixer_init):
                    fixer[0] = float(self.data[i][4])
                    fixer[1] = float(self.data[i][5])
                    fixer[2] = float(self.data[i][6])
                    fixer_init = True
                
                self.times.append(float(self.data[i][0]))
                self.xs.append(float(self.data[i][4]) - fixer[0])
                self.ys.append(float(self.data[i][5]) - fixer[1])
                self.zs.append(float(self.data[i][6]) - fixer[2])


    def remove_outliers(self):
        # Filter for ooutliers
        # Remove any data points which requires a speed of 10 m/s or more
        for i in range(2, len(self.xs)):
            time_step = self.times[i] - self.times[i-1]
            dist = math.sqrt(math.pow(self.xs[i] - self.xs[i-1], 2) + math.pow(self.ys[i] - self.ys[i-1], 2))
            if (dist/time_step >= 50):
                print("Outlier found! ", i, dist, time_step)
            else:
                self.x_filtered.append(self.xs[i])
                self.y_filtered.append(self.ys[i])
                self.z_filtered.append(self.zs[i])

        print("Amount of outliers:", len(self.xs)-len(self.x_filtered))


    def plot(self):
        self._plotter(self.xs, self.ys, self.zs)


    def plot_filtered(self):
        self._plotter(self.x_filtered, self.y_filtered, self.z_filtered)


    def _plotter(self, xs, ys, zs):
        xs = np.array(xs)
        ys = np.array(ys)
        zs = np.array(zs)

        print(len(xs), len(ys), len(zs))

        # Init figure
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

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
        
        plt.show()



ro = Remove_GNSS_Outliers()
ro.load_data()
ro.remove_outliers()
ro.plot()