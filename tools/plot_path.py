'''
Plots a path
'''

import numpy as np
import matplotlib.pyplot as plt
import argparse
from pathlib import Path
import json
from pprint import pprint

def plot_path(path):
    '''
    Plots a path in 2D coords, x upwards
    '''

    # Create axes and plot
    fig = plt.figure()
    ax = fig.gca()

    # Plot the path
    ax.plot(path[:,0], path[:,1])

    # Plot the start and end points
    ax.plot(path[0, 0], path[1, 1], 'gx')
    ax.plot(path[-1, 0], path[-1, 1], 'rx')

    ax.set_xlabel('X_Lm [m]')
    ax.set_ylabel('Y_Lm [m]')

    ax.set_aspect('equal', 'box')

    plt.show()

def load_path(path):
    '''
    Load a path JSON file
    '''
    with open(path, 'r') as f:
        path = json.loads(f.read())

    path = np.array(path['points_m'])

    return path

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot a path')
    parser.add_argument(
        'path_path',
        metavar='PATH', 
        type=str, 
        nargs=1,
        help='Path to the path JSON file to plot'
    )

    args = parser.parse_args()

    path = load_path(Path(args.path_path[0]))

    plot_path(path)
