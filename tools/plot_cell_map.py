'''
Plots a grid map
'''

from mpl_toolkits.mplot3d import Axes3D 
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.colors import LinearSegmentedColormap
import numpy as np
from pathlib import Path
from pprint import pprint
import argparse
import json
import copy

from plot_path import load_path, plot_path
from cell_map import CellMap

def plot_grid_map(map, paths, plan_report = None, show_leaves=False):
    '''
    Plots the given grid map in an interactive way.
    '''

    print(f'Showing map {map["path"]}')
    print('Press n to view next layer, b for previous layer')

    plots = {}
    selected_layer = map['cm'].layers[0]

    # Create the figure and axis
    fig = plt.figure()
    if map['data_type'] == 'CostMap':
        ax = fig.gca()
    else:
        ax = fig.gca(projection='3d')

    # Calculate range of X, Y, and Z data for the first layer
    x_range = (map['x_coords'].max() - map['x_coords'].min())
    y_range = (map['y_coords'].max() - map['y_coords'].min())
    z_range = (
        np.nanmax(map['cm'].data[selected_layer]) - np.nanmin(map['cm'].data[selected_layer])
    )

    # Normalize by x_range, to create an aspect ratio in which units have the
    # same physical size
    aspect_ratio = [1, y_range/x_range, z_range/x_range]
    if not map['data_type'] == 'CostMap':
        ax.set_box_aspect(aspect_ratio)
    
    # Set color maps
    cmap = copy.copy(cm.get_cmap('viridis'))
    cmap.set_bad('white')
    if map['data_type'] == 'CostMap':
        # Set color maps
        cmap = copy.copy(cm.get_cmap('viridis'))
        cmap.set_over('red')
        cmap.set_under('white')

    # Draw all layers, but only the selected one is visible
    for layer in map['cm'].layers:
        if map['data_type'] == 'CostMap':
            plots[layer] = map['cm'].plot(ax=ax, cmap=cmap, vmin=0.0, vmax=1.0)
            for i, path in enumerate(paths):
                if i == 0:
                    plot_path(path, ax=ax, linespec='-k')
                else:
                    plot_path(path, ax=ax, linespec='-b')
                    
            if plan_report is not None:
                plot_plan_report(plan_report, ax, show_leaves)
        else:
            plots[layer] = ax.plot_surface(
                map['x_grid'], map['y_grid'], map['cm'].data[layer],
                cmap=cmap,
                vmin=np.nanmin(map['cm'].data[layer]),
                vmax=np.nanmax(map['cm'].data[layer]),
                linewidth=0,
                antialiased=True
            )
        if layer is not selected_layer:
            plots[layer].set_visible(False)

    plt.title(selected_layer)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')

    if map['data_type'] == 'CostMap':
        color_bar = plt.colorbar(plots[selected_layer], extend='both')
        color_bar.set_ticks([0.0, 0.2, 0.4, 0.6, 0.8, 1.0])
        color_bar.set_ticklabels(['Unknown', 0.2, 0.4, 0.6, 0.8, 'Unsafe'])
        color_bar.set_label('Cost')
    else:
        plt.colorbar(plots[selected_layer])

    # Function to change the selected layer
    def key_press_event(event):
        if len(plots) == 1:
            return

        # define selected_layer as non-local
        nonlocal selected_layer

        n = None
        p = None
        for i, l in enumerate(map['cm'].layers):
            if l == selected_layer:
                if i == 0:
                    p = map['cm'].layers[-1]
                else:
                    p = map['cm'].layers[i - 1]
                if i == len(map['cm'].layers) - 1:
                    n = map['cm'].layers[0]
                else:
                    n = map['cm'].layers[i + 1]

        if event.key == 'n':
            plots[selected_layer].set_visible(False)
            plots[n].set_visible(True)
            selected_layer = n
            plt.title(selected_layer)
        elif event.key == 'b':
            plots[selected_layer].set_visible(False)
            plots[p].set_visible(True)
            selected_layer = p
            plt.title(selected_layer)
        else:
            return
        plt.draw()

    plt.connect('key_press_event', key_press_event)
    plt.show()

def load_map(path):
    '''
    Load a grid map JSON file from the given path
    '''
    print(f'Loading map from {path}')

    with open(path, 'r') as f:
        raw = json.loads(f.read())

    # Cost maps etc. contain parameters in them as well, so we have to actually
    # find the map bit, which will be named `map`.
    if 'map' in raw and 'data' not in raw:
        # Load the cell map
        cm = CellMap.from_raw_dict(raw['map'])
    else:
        cm = CellMap.from_raw_dict(raw)


    # Create container
    map = dict()
    map['cm'] = cm
    map['path'] = path

    # Convert data from Rust-types to numpy types based on the data type name
    # given in the cost map
    if 'cost_map_params' in raw:
        for layer in map['cm'].layers:
            map['cm'].data[layer] = conv_cost_map_data(map['cm'].data[layer])
        map['data_type'] = 'CostMap'
    else:
        for layer in map['cm'].layers:
            map['cm'].data[layer] = conv_opt_f64(map['cm'].data[layer])
        map['data_type'] = 'TerrainMap'

    # Calculate meshgrids for easy plotting
    axis_length = map['cm'].cell_bounds * map['cm'].cell_size
    print(axis_length)
    map['x_coords'] = np.linspace(axis_length[0][0], axis_length[0][1], map['cm'].num_cells[1])
    map['y_coords'] = np.linspace(axis_length[1][0], axis_length[1][1], map['cm'].num_cells[0])
    map['x_grid'], map['y_grid'] = np.meshgrid(map['x_coords'], map['y_coords'])

    print(f'Map of {map["cm"].num_cells} cells loaded')

    return map

def conv_cost_map_data(data):
    '''
    Converts cost map data (rov_lib::auto::map::cost_map::CostMapData) values
    into floating point values, with CostMapData::None being NaN and
    CostMapData::Unsafe being 1.1 (above max cost of 1.0).
    '''

    def conv(val):
        if isinstance(val, dict):
            if 'Cost' in val:
                return val['Cost']
            elif 'Unsafe' in val:
                return val['Unsafe']
        elif isinstance(val, str):
            if val == 'None':
                return np.nan
            else:
                raise RuntimeError(f'Unknown CostMapData variant {val}')

    return np.vectorize(conv)(data)

def conv_opt_f64(data):
    '''
    Converts core::option::Option<f64> values into floating points values, with
    None being np.nan
    '''

    def conv(val):
        if val is None:
            return np.nan
        else:
            return val

    return np.vectorize(conv)(data)

def plot_plan_report(plan_report, ax, show_leaves = False):
    '''
    Plots a PathPlannerReport file to the given axis
    '''

    def plot_node(node, ax):
        if node['node'] is not None:
            if len(node['children']) == 0:
                if show_leaves:
                    linespec = ':k'
                else:
                    return
            else:
                linespec = ':b'
            plot_path(np.array(node['node']['path']['points_m']), ax=ax, linespec=linespec)

        for child in node['children']:
            plot_node(child, ax)
    
    plot_node(plan_report['tree'], ax)
    

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot a grid map')
    parser.add_argument(
        'grid_map_path',
        metavar='MAP', 
        type=str, 
        nargs=1,
        help='Path to the grid map JSON file to plot'
    )
    parser.add_argument(
        'paths',
        metavar='PATHS',
        nargs='*',
        help='Path to one or more JSON files describing paths'
    )
    parser.add_argument(
        '--plan_report',
        metavar='PLANRPT',
        nargs='?',
        help='Path to PathPlannerReport file.'
    )
    parser.add_argument(
        '--show_leaves',
        action='store_true',
        help='If a PathPlannerReport is provided, show the leaves of any planning'
    )

    args = parser.parse_args()

    map = load_map(Path(args.grid_map_path[0]))
    
    if args.plan_report is not None:
        with open(args.plan_report) as f:
            plan_report = json.load(f)
    else:
        plan_report = None

    paths = []
    for path in args.paths:
        loaded_path = load_path(path)
        if isinstance(loaded_path, list):
            paths.extend(loaded_path)
        else:
            paths.append(loaded_path)

    plot_grid_map(map, paths, plan_report=plan_report, show_leaves=args.show_leaves)

