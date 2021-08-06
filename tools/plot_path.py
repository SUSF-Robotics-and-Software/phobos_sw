'''
Plots a path
'''

import numpy as np
import matplotlib.pyplot as plt
import argparse
from pathlib import Path
import json
from pprint import pprint

def plot_path(path, items=None, ax=None, linespec='-b', linecolor=None, startspec='gx', endspec='rx'):
    '''
    Plots a path in 2D coords, x upwards
    '''

    # Create axes and plot if needed
    if ax is None:
        fig = plt.figure()
        ax = fig.gca()
        ax.set_xlabel('X_Lm [m]')
        ax.set_ylabel('Y_Lm [m]')
        ax.set_aspect('equal', 'box')
        show = True
    else:
        show = False

    # Set the items up
    if items is None:
        items = [None, None, None]
    
    # Plot the path
    if items[0] is None:
        items[0] = ax.plot(path[:,0], path[:,1], linespec)[0]
    else:
        items[0].set_data(path[:, 0], path[:, 1])
    
    if linecolor is not None:
        items[0].set_color(linecolor)

    # Plot the start and end points
    if items[1] is None:
        items[1] = ax.plot(path[0, 0], path[0, 1], startspec)[0]
    else:
        items[1].set_data(path[0, 0], path[0, 1])

    if items[2] is None:
        items[2] = ax.plot(path[-1, 0], path[-1, 1], endspec)[0]
    else:
        items[2].set_data(path[-1, 0], path[-1, 1])

    if show:
        plt.show()

    return items

def conv_path(path):
    '''
    Converts from the rust Path struct into a simple numpy array
    '''

    return np.array(path['points_m'])

def load_path(path):
    '''
    Load a path JSON file, returning either a single or many paths
    '''
    with open(path, 'r') as f:
        data = json.loads(f.read())

    if isinstance(data, list):
        return [conv_path(path) for path in data]
    else:
        return conv_path(data)

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
