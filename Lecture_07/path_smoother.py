
import numpy as np
import matplotlib.pyplot as plt
import csv
from mpl_toolkits.mplot3d import Axes3D

class path_smoother:
    def __init__(self):
        pass

    def rdp(self, points, epsilon):
        points = np.array(points)
        start, end = points[0], points[-1]
        dmax = 0
        index = 0
        for i in range(1, len(points) - 1):
            d = self.point_line_distance(points[i], start, end)
            if d > dmax:
                index = i
                dmax = d

        if dmax > epsilon:
            # Recursively simplify
            left = self.rdp(points[:index+1], epsilon)
            right = self.rdp(points[index:], epsilon)
            return left[:-1] + right
        else:
            return [tuple(start), tuple(end)]


    def point_line_distance(self, point, start, end):
        # Calculate distance of `point` from the line defined by `start` and `end`
        if np.array_equal(start, end):
            return np.linalg.norm(point - start)
        else:
            return np.linalg.norm(np.cross(end - start, start - point)) / np.linalg.norm(end - start)

    def load_data(self, file_name):

        x_old = []
        y_old = []
        z_old = []

        with open(file_name, newline='') as csvfile:
            points = list(csv.reader(csvfile))

        for i in range(1, len(points)):
            x_old.append(float(points[i][7]))
            y_old.append(float(points[i][8]))
            z_old.append(float(points[i][9]))

        points = []
        for i in range(len(x_old)):
            points.append((x_old[i], y_old[i], z_old[i]))

        return x_old, y_old, z_old, points

    def simplify_route(self, file_name, deviation, max_points):
        """
        Simplify a 3D route to a maximum of `max_points`.
        Args:
            points (list of tuples): Original waypoints.
            max_points (int): Desired maximum number of waypoints.
        Returns:
            list of tuples: Simplified waypoints.
        """

        # Loading data
        x_old, y_old, z_old, points = self.load_data(file_name)

        # Start with a very small epsilon and increase until we reach the target count
        if len(points) <= max_points:
            return points  # No need to simplify
        
        low, high = 0, 1000  # Initial bounds for epsilon
        best_result = points

        while low <= high:
            mid = (low + high) / 2
            simplified = self.rdp(points, mid)
            
            if len(simplified) > max_points:
                low = mid + 0.01
            else:
                best_result = simplified
                high = mid - 0.01
        
        # Adjust to exact count by sampling if necessary
        if len(best_result) > max_points:
            indices = np.round(np.linspace(0, len(best_result) - 1, max_points)).astype(int)
            best_result = [best_result[i] for i in indices]
        
        x = []
        y = []
        z = []

        for i in range(len(best_result)):
            x.append(best_result[i][0])
            y.append(best_result[i][1])
            z.append(best_result[i][2])

        return x_old, y_old, z_old, x, y, z




if __name__ == '__main__':
    smooth = path_smoother()
    x_old, y_old, z_old, x_simp, y_simp, z_simp = smooth.simplify_route(file_name='data1.csv', max_points=100)
    

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_title("Orginal Route Plan")
    ax.plot(x_old, y_old, z_old)

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_title("Simplified Route Plan")
    ax.plot(x_simp, y_simp, z_simp)


    print("Length of old route plan: ", len(x_old))
    print("Length of new route plan: ", len(x_simp))

    plt.show()