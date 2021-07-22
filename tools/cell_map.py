'''
Generic CellMap specific helper class/utility
'''

import numpy as np
import json
import typing
import matplotlib.pyplot as plt

class CellMap:
    data: typing.Dict[str, np.ndarray]
    layers: typing.List[str]
    cell_size: np.ndarray
    cell_bounds: typing.Tuple[np.ndarray, np.ndarray]
    num_cells: np.ndarray
    extents: np.ndarray
    cell_boundary_precision: float
    from_parent: np.ndarray
    to_parent: np.ndarray


    @staticmethod
    def load(path):
        '''
        Loads a CellMap from the given path, expects a JSON file.
        '''

        # Read the data
        with open(path, 'r') as f:
            return CellMap.from_raw_dict(json.load(f))
    
    @staticmethod
    def from_raw_dict(raw, path=''):
        '''
        Loads a CellMap from a raw dictionary, i.e. from deserialised JSON.
        '''

        cm = CellMap()

        # Load metadata
        cm.path = path
        cm.layers = raw['layers']
        cm.cell_size = np.array(raw['cell_size'])
        cm.cell_bounds = np.array([raw['cell_bounds']['x'], raw['cell_bounds']['y']])
        cm.num_cells = np.array([
            cm.cell_bounds[1][1] - cm.cell_bounds[1][0],
            cm.cell_bounds[0][1] - cm.cell_bounds[0][0] 
        ])
        cm.cell_boundary_precision = np.array(raw['cell_boundary_precision'])
        cm.from_parent = np.array(raw['from_parent_matrix']).reshape((3, 3))
        cm.to_parent = np.linalg.inv(cm.from_parent)

        # Calculate extents of map
        extents = np.array([
            [cm.cell_bounds[0][0], cm.cell_bounds[1][0]], 
            [cm.cell_bounds[0][1], cm.cell_bounds[1][0]], 
            [cm.cell_bounds[0][0], cm.cell_bounds[1][1]], 
            [cm.cell_bounds[0][1], cm.cell_bounds[1][1]], 
        ])
        cm.extents = cm.transform_to_parent(extents)

        # Load each layer in turn, reshaping as needed
        cm.data = dict()
        for layer, data in zip(cm.layers, raw['data']):
            if data['dim'][0] != cm.num_cells[0] or data['dim'][1] != cm.num_cells[1]:
                raise RuntimeError(f'Data in cell map file is of wrong shape. Expected {cm.num_cells} but got {data["dim"]}')
            cm.data[layer] = np.array(data['data']).reshape(cm.num_cells)

        return cm

    def transform_to_parent(self, points: np.ndarray):
        '''
        Converts the given point(s) from the map frame to the parent frame.

        Points should be an (N, 2) dimension array.
        '''
        n = np.shape(points)[0]
        dehomog = lambda x: x[:-1]/x[-1]
        homog = np.ones((n, 3))
        homog[:,:-1] = points
        homog = homog @ self.to_parent
        return np.array([dehomog(x) for x in homog])

    def plot(self, name = None, ax = None, cmap='viridis', parent_relative = True, show_grid=False, vmin=None, vmax=None):
        '''
        Plots the given CellMap

        Arguments:
            map: The CellMap to plot
            name: The name to place in the title
            ax: The axis to plot onto, or None if we should create a new figure
            parent_relative: True if the map should be plotted relative to the parent
        '''
        
        if ax is None:
            fig, ax = plt.subplots()
        else:
            fig = None

        # Map-relative origin and axes directions
        origin = np.array([0.0, 0.0])
        x_dir = np.array([1.0, 0.0])
        y_dir = np.array([0.0, 1.0])

        # Map-relative limits
        x_lims = [-0.5, self.num_cells[0] + 0.5]
        y_lims = [-0.5, self.num_cells[1] + 0.5]

        # Setup the axis grid
        if parent_relative:
            # Get the grid
            grid = ax.add_artist(self._get_parent_rel_grid(show_grid=show_grid, cmap=cmap, vmin=vmin, vmax=vmax))

            # Update self origin
            origin = self.transform_to_parent(origin.reshape((1, 2))).reshape((2,))
            x_dir = self.transform_to_parent(x_dir.reshape((1, 2))).reshape((2,)) - origin
            y_dir = self.transform_to_parent(y_dir.reshape((1, 2))).reshape((2,)) - origin

            plot_bounds = np.max([self.cell_size[0] * 0.5, self.cell_size[1] * 0.5])

            # Update limits
            ext_plus_origin_x = np.append(self.extents[:,0], origin[0])
            ext_plus_origin_y = np.append(self.extents[:,1], origin[1])
            x_lims = [np.min(ext_plus_origin_x) - plot_bounds, np.max(ext_plus_origin_x) + plot_bounds]
            y_lims = [np.min(ext_plus_origin_y) - plot_bounds, np.max(ext_plus_origin_y) + plot_bounds]

        else:
            # Include the end line in the ticks
            x_ticks = range(self.num_cells[0] + 1)
            y_ticks = range(self.num_cells[1] + 1)
            ax.set_xticks(x_ticks)
            ax.set_yticks(y_ticks)
            ax.grid(True)

        # Plot origin and directions
        ax.plot(origin[0], origin[1], '.k')
        ax.quiver(*origin, x_dir[0], x_dir[1], color='r', angles='xy', scale_units='xy', scale=1)
        ax.quiver(*origin, y_dir[0], y_dir[1], color='g', angles='xy', scale_units='xy', scale=1)

        # Set limits
        ax.set_xlim(x_lims)
        ax.set_ylim(y_lims)
        ax.set_aspect('equal', 'box')

        if name is not None:
            ax.set_title(name)
        else:
            ax.set_title(self.path)

        if fig is not None:
            plt.show()

        return grid

    def _get_parent_rel_grid(self, show_grid=False, cmap='viridis', vmin=None, vmax=None):
        '''
        Gets the parent-relative grid as a matplotlib.collections.LineCollection
        '''

        # Create mesh grid points by transforming each meshgrid point into the
        # parent frame
        mesh_x, mesh_y = np.meshgrid(
            np.array(range(self.cell_bounds[0][0], self.cell_bounds[0][1] + 1)),
            np.array(range(self.cell_bounds[1][0], self.cell_bounds[1][1] + 1))
        )
        mesh_shape = mesh_x.shape
        mesh_points = np.vstack([mesh_x.ravel(), mesh_y.ravel()]).T
        mesh_points = self.transform_to_parent(mesh_points)
        mesh_x, mesh_y = [mesh_points[:,0].reshape(mesh_shape), mesh_points[:,1].reshape(mesh_shape)]
        
        mesh = plt.pcolormesh(
            mesh_x, mesh_y, self.data[self.layers[0]], 
            shading='flat', 
            edgecolors='grey' if show_grid else None, 
            cmap=cmap,
            linewidth=0.1, 
            zorder=-1.0,
            vmin=vmin,
            vmax=vmax,
        )

        return mesh
