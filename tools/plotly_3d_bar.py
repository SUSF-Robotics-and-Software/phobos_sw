'''
Functions for creating a 3D plotly bar chart from x, y, z, meshgrid data.

Based on https://chart-studio.plotly.com/~empet/15255/plotly-3d-barchart-or-histogram3d-for-2d/#/
'''

import numpy as np
import plotly.graph_objects as go

def bar_data(position3d, size=(1,1,1)):
    # position3d - 3-list or array of shape (3,) that represents the point of coords (x, y, 0), where a bar is placed
    # size = a 3-tuple whose elements are used to scale a unit cube to get a paralelipipedic bar
    # returns - an array of shape(8,3) representing the 8 vertices of  a bar at position3d
    
    bar = np.array([[0, 0, 0],
                    [1, 0, 0],
                    [1, 1, 0],
                    [0, 1, 0],
                    [0, 0, 1],
                    [1, 0, 1],
                    [1, 1, 1],
                    [0, 1, 1]], dtype=float) # the vertices of the unit cube
   
    bar *= np.asarray(size)# scale the cube to get the vertices of a parallelipipedic bar
    bar += np.asarray(position3d) #translate each  bar on the directio OP, with P=position3d
    return bar

def triangulate_bar_faces(positions, sizes=None):
    # positions - array of shape (N, 3) that contains all positions in the plane z=0, where a histogram bar is placed 
    # sizes -  array of shape (N,3); each row represents the sizes to scale a unit cube to get a bar
    # returns the array of unique vertices, and the lists i, j, k to be used in instantiating the go.Mesh3d class

    if sizes is None:
        sizes = [(1,1,1)]*len(positions)
    else:
        if isinstance(sizes, (list, np.ndarray)) and len(sizes) != len(positions):
            raise ValueError('Your positions and sizes lists/arrays do not have the same length')
            
    all_bars = [bar_data(pos, size)  for pos, size in zip(positions, sizes) if size[2]!=0]
    p, q, r = np.array(all_bars).shape
    
    # extract unique vertices from the list of all bar vertices
    vertices, ixr = np.unique(np.array(all_bars).reshape(p*q, r), return_inverse=True, axis=0)
    #for each bar, derive the sublists of indices i, j, k assocated to its chosen  triangulation
    I = []
    J = []
    K = []
     
    for k in range(len(all_bars)):
        I.extend(np.take(ixr, [8*k, 8*k+2,8*k, 8*k+5,8*k, 8*k+7, 8*k+5, 8*k+2, 8*k+3, 8*k+6, 8*k+7, 8*k+5])) 
        J.extend(np.take(ixr, [8*k+1, 8*k+3, 8*k+4, 8*k+1, 8*k+3, 8*k+4, 8*k+1, 8*k+6, 8*k+7, 8*k+2, 8*k+4, 8*k+6])) 
        K.extend(np.take(ixr, [8*k+2, 8*k, 8*k+5, 8*k, 8*k+7, 8*k, 8*k+2, 8*k+5, 8*k+6, 8*k+3, 8*k+5, 8*k+7]))  
   
    return  vertices, I, J, K  #triangulation vertices and I, J, K for mesh3d
    
def get_plotly_mesh3d(x, y, bins=[5,5], bargap=0.05, range_extent=0.2):
    # x, y- array-like of shape (n,), defining the x, and y-ccordinates of data set for which we plot a 3d hist
    hist, xedges, yedges = np.histogram2d(x, y, 
                                          bins=bins, 
                                          range=[[x.min()-range_extent, x.max()+range_extent],
                                                 [y.min()-range_extent, y.max()+range_extent]])
    xsize = xedges[1]-xedges[0]-bargap
    ysize = yedges[1]-yedges[0]-bargap
    xe, ye= np.meshgrid(xedges[:-1], yedges[:-1])
    ze = np.zeros(xe.shape)

    positions =np.dstack((xe, ye, ze))
    m, n, p = positions.shape
    positions = positions.reshape(m*n, p)
    sizes = np.array([(xsize, ysize, h) for h in hist.flatten()])
    vertices, I, J, K  = triangulate_bar_faces(positions, sizes=sizes)
    X, Y, Z = vertices.T
    return X, Y, Z, I, J, K

def plotly_3d_bar(x, y, z, bar_gap=0):
    '''
    Returns a plotly figure with a 3D bar chart of the given x, y, z meshgrid data.
    '''

    X, Y, Z, I, J, K = get_plotly_mesh3d(x, y, bins=x.shape, bargap=bar_gap)

    mesh3d = go.Mesh3d(x=X, y=Y, z=Z, i=I, j=J, k=K)

    return go.Figure(data=[mesh3d])