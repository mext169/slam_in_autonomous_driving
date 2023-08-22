## coding=UTF-8

import sys
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Please input valid file')
        exit(1)
    else:
        path = sys.argv[1]
        path_data = np.loadtxt(path)
        data_x = path_data[:, 0]
        data_y = path_data[:, 1]
        data_z = path_data[:, 2]

        fig = plt.figure()
        ax = fig.gca(projection='3d')

        ax.set_title("3d_traj")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")

        figure = ax.plot(data_x, data_y, data_z, c='r')
        plt.show()
