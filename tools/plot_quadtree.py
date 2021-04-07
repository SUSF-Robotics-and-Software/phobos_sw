'''
Used to debug quadtree implementation
'''

import numpy as np
import matplotlib.pyplot as plt
import argparse
from pathlib import Path
import json
from pprint import pprint

def plot_quadtree(qt):
    '''
    Plots the quadtree
    '''

    fig = plt.figure()
    ax = fig.gca()

    draw_qt(qt, ax)

    ax.set_xlabel('X_Lm [m]')
    ax.set_ylabel('Y_Lm [m]')

    ax.set_aspect('equal', 'box')

    plt.show()

def draw_qt(qt, ax, c='k', lw=1, **kwargs):

    if len(qt['points']) > 0:
        sc = ax.scatter(
            [p[0] for p in qt['points']], 
            [p[1] for p in qt['points']],
            s=4
        )
        draw_quad(qt['boundary'], ax, c, lw, **kwargs)
    else:
        #draw_quad(qt['boundary'], ax, c, lw, **kwargs)
        pass

    if qt['north_west'] is not None:
        draw_qt(qt['north_west'], ax, c, lw, **kwargs)
    if qt['north_east'] is not None:
        draw_qt(qt['north_east'], ax, c, lw, **kwargs)
    if qt['south_west'] is not None:
        draw_qt(qt['south_west'], ax, c, lw, **kwargs)
    if qt['south_east'] is not None:
        draw_qt(qt['south_east'], ax, c, lw, **kwargs)

def draw_quad(quad, ax, col='k', lw=1, **kwargs):
    c = quad['centre']
    hw = quad['half_width']
    ax.plot(
        [c[0] - hw, c[0] + hw, c[0] + hw, c[0] - hw, c[0] - hw],
        [c[1] - hw, c[1] - hw, c[1] + hw, c[1] + hw, c[1] - hw], 
        c=col, lw=lw, **kwargs
    )

def load_quadtree(path):
    '''
    Load a quadtree JSON file
    '''
    with open(path, 'r') as f:
        qt = json.loads(f.read())

    return qt

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot a quadtree')
    parser.add_argument(
        'qt_path',
        metavar='PATH', 
        type=str, 
        nargs=1,
        help='Path to the quadtree JSON file to plot'
    )

    args = parser.parse_args()

    qt = load_quadtree(Path(args.qt_path[0]))

    plot_quadtree(qt)