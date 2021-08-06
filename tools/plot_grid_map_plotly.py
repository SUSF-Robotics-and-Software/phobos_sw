'''
Plots a grid map using plotly rather than matplotlib
'''

import plotly.graph_objects as go
import numpy as np
from pathlib import Path
from pprint import pprint
import argparse
from plot_grid_map import load_map
from plot_path import load_path
from plotly_3d_bar import plotly_3d_bar

def plot_grid_map(map, paths):
    '''
    Plots the given grid map and optional paths using plotly
    '''

    sorted_layers = sorted(map['layer_map'].items(), key=lambda item: item[1])
    sorted_layers = [i[0] for i in sorted_layers]
    initial_layer_idx = map['layer_map'][sorted_layers[0]]

    print(sorted_layers)

    fig = go.Figure(data=[
        go.Surface(x=map['x_grid'], y=map['y_grid'], z=map['data'][initial_layer_idx])
    ])

    options_button_y = 1.05
    options_label_y = 1.07

    fig.update_layout(
        title=f'Grid Map {map["path"]}', 
        scene={'aspectmode': 'data'},
        updatemenus = [
            dict(
                buttons = [dict(
                    args=['z', [map['data'][map['layer_map'][layer]]]],
                    label=layer,
                    method='restyle'
                ) 
                for layer in sorted_layers],
                direction='down',
                pad={"r": 10, 't': 10},
                showactive=True,
                x=0.01,
                xanchor='left',
                y=options_button_y,
                yanchor='top'
            ),
            dict(
                buttons= [dict(
                    args=['surfacecolor', [map['data'][map['layer_map'][layer]]]],
                    label=layer,
                    method='restyle'
                )
                for layer in sorted_layers],
                direction='down',
                pad={'r': 10, 't': 10},
                showactive=True,
                x=0.1,
                xanchor='left',
                y=options_button_y,
                yanchor='top'
            )
        ]
    )
    fig.update_layout(
        annotations=[
            dict(text='Surface', x=0.01, xref='paper', y=options_label_y, yref='paper', showarrow=False),
            dict(text='Colour', x=0.1, xref='paper', y=options_label_y, yref='paper', showarrow=False)
        ]
    )
    fig.show()

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

    args = parser.parse_args()

    map = load_map(Path(args.grid_map_path[0]))
    paths = [load_path(path) for path in args.paths]

    plot_grid_map(map, paths)