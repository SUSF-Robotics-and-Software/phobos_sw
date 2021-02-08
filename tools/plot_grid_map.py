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

def plot_grid_map(map):
    '''
    Plots the given grid map in an interactive way.
    '''

    print(f'Showing map {map["path"]}')
    print('Press n to view next layer, b for previous layer')

    plots = {}
    sorted_layers = sorted(map['layer_map'].items(), key=lambda item: item[1])
    sorted_layers = [i[0] for i in sorted_layers]
    selected_layer = sorted_layers[0]

    # Create the figure and axis
    fig = plt.figure()
    if map['is_cost_map']:
        ax = fig.gca()
    else:
        ax = fig.gca(projection='3d')

    # Calculate range of X, Y, and Z data for the first layer
    x_range = (map['x_coords'].max() - map['x_coords'].min())
    y_range = (map['y_coords'].max() - map['y_coords'].min())
    z_range = (np.nanmax(map['data'][0]) - np.nanmin(map['data'][0]))

    # Normalize by x_range, to create an aspect ratio in which units have the
    # same physical size
    aspect_ratio = [1, y_range/x_range, z_range/x_range]
    if not map['is_cost_map']:
        ax.set_box_aspect(aspect_ratio)
    
    # Set color maps
    cmap = copy.copy(cm.get_cmap('viridis'))
    cmap.set_bad('white')
    if map['is_cost_map']:
        # Set color maps
        cmap = copy.copy(cm.get_cmap('viridis'))
        cmap.set_over('red')
        cmap.set_under('white')

            
    # Draw all layers, but only the selected one is visible
    for (layer, idx) in map['layer_map'].items():
        if map['is_cost_map']:
            plots[layer] = ax.pcolormesh(
                map['x_grid'], map['y_grid'], map['data'][idx],
                cmap=cmap,
                shading='auto',
                vmin=0.0,
                vmax=1.0
            )
        else:
            plots[layer] = ax.plot_surface(
                map['x_grid'], map['y_grid'], map['data'][idx],
                cmap=cmap,
                vmin=np.nanmin(map['data'][idx]),
                vmax=np.nanmax(map['data'][idx]),
                linewidth=0,
                antialiased=True
            )
        if layer is not selected_layer:
            plots[layer].set_visible(False)

    plt.title(selected_layer)

    if map['is_cost_map']:
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
        for i, l in enumerate(sorted_layers):
            if l == selected_layer:
                if i == 0:
                    p = sorted_layers[-1]
                else:
                    p = sorted_layers[i - 1]
                if i == len(sorted_layers) - 1:
                    n = sorted_layers[0]
                else:
                    n = sorted_layers[i + 1]

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
        map = json.loads(f.read())

    # Change array-like data into numpy arrays
    map['path'] = path
    map['data'] = np.reshape(map['data']['data'], map['data']['dim'])
    map['num_cells'] = np.array(map['num_cells'])
    map['cell_size'] = np.array(map['cell_size'])
    map['centre_position'] = np.array(map['centre_position'])

    # Convert data from Rust-types to numpy types based on the data type name
    # given in the cost map
    if map['data_type'] == 'rov_lib::auto::map::cost_map::CostMapData':
        map['data'] = conv_cost_map_data(map['data'])
        map['is_cost_map'] = True
    else:
        map['data'] = conv_opt_f64(map['data'])
        map['is_cost_map'] = False

    # Calculate meshgrids for easy plotting
    axis_length = map['num_cells'] * map['cell_size']
    ul_pos = map['centre_position'] + 0.5 * axis_length
    lr_pos = map['centre_position'] - 0.5 * axis_length
    map['x_coords'] = np.arange(ul_pos[0], lr_pos[0], -map['cell_size'][0])
    map['y_coords'] = np.arange(ul_pos[1], lr_pos[1], -map['cell_size'][1])
    map['x_grid'], map['y_grid'] = np.meshgrid(map['x_coords'], map['y_coords'])

    print(f'Map of shape {map["data"].shape} loaded')

    return map

def conv_cost_map_data(data):
    '''
    Converts cost map data (rov_lib::auto::map::cost_map::CostMapData) values
    into floating point values, with CostMapData::None being NaN and
    CostMapData::Unsafe being 2.0 (above max cost of 1.0).
    '''

    def conv(val):
        if isinstance(val, dict):
            return val['Cost']
        elif isinstance(val, str):
            if val == 'None':
                return np.nan
            elif val == 'Unsafe':
                return 1.1
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

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot a grid map')
    parser.add_argument(
        'grid_map_path',
        metavar='PATH', 
        type=str, 
        nargs=1,
        help='Path to the grid map JSON file to plot'
    )

    args = parser.parse_args()

    map = load_map(Path(args.grid_map_path[0]))

    plot_grid_map(map)

