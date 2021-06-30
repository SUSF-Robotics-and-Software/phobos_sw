'''
Generic CellMap specific helper class/utility
'''

import numpy as np
import json
import typing

class CellMap:
    data: typing.Dict[str, np.ndarray]
    layers: typing.List[str]
    cell_size: np.ndarray
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
            raw = json.load(f)

        return CellMap.from_raw_json(raw)

    @staticmethod
    def from_raw_json(raw):
        '''
        Creates a new CellMap from the raw JSON data
        '''

        cm = CellMap()

        # Load metadata
        cm.layers = raw['layers']
        cm.cell_size = np.array(raw['cell_size'])
        cm.num_cells = np.array(raw['num_cells'])
        cm.cell_boundary_precision = np.array(raw['cell_boundary_precision'])
        cm.from_parent = np.array(raw['from_parent_matrix']).reshape((3, 3))
        cm.to_parent = np.linalg.inv(cm.from_parent)

        # Calculate extents of map
        extents = np.array([
            [0.0, 0.0], 
            [cm.num_cells[0], 0.0],
            [0.0, cm.num_cells[1]], 
            [cm.num_cells[0], cm.num_cells[1]], 
        ])
        cm.extents = cm.transform_to_parent(extents)

        # Load each layer in turn, reshaping as needed
        cm.data = dict()
        for layer, data in zip(cm.layers, raw['data']):
            if data['dim'][0] != cm.num_cells[1] or data['dim'][1] != cm.num_cells[0]:
                raise RuntimeError('Data in cell map file is of wrong shape')
            cm.data[layer] = np.array(data['data']).reshape(data['dim'])

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
