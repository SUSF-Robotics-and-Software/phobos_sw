
import argparse
import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_point_cloud(point_cloud):
    '''
    Plots the point cloud
    '''

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Only plot 1000 random points since we've got a lot to plot
    sample_idxs = np.random.choice(point_cloud.shape[0], size=1000)
    sample = point_cloud[sample_idxs]

    ax.scatter(sample[:,0], sample[:,1], sample[:,2], 'k')

    ax.set_xlabel('X_RB [m]')
    ax.set_ylabel('Y_RB [m]')
    ax.set_zlabel('Z_RB [m]')

    set_axes_equal(ax)

    plt.show()

def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

def load_point_cloud(path):
    '''
    Loads the point cloud from the given path
    '''

    with open(path, 'r') as f:
        data = json.loads(f.read())

    return np.array(data)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot a grid map')
    parser.add_argument(
        'point_cloud_path',
        metavar='MAP', 
        type=str, 
        nargs=1,
        help='Path to the point cloud JSON file to plot'
    )

    args = parser.parse_args()

    point_cloud = load_point_cloud(args.point_cloud_path[0])

    plot_point_cloud(point_cloud)