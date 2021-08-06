'''
Provides a live telemetry view of the autonomy system, including:
    - Estimated rover position and attitude
    - Global terrain map
    - Path
    - TrajCtrl data - lat/long/head errors
'''

import zmq
from pprint import pprint
import json
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import _pylab_helpers
from matplotlib.rcsetup import interactive_bk as _interactive_bk
from plot_path import plot_path, conv_path
from plot_cell_map import conv_cost_map_data
from cell_map import CellMap
import copy

matplotlib.use('Qt5agg')

# CONSTANTS
TM_SUB_ENDPOINT = 'tcp://localhost:5030'
MIN_PLOT_BOX_SIZE = 1.0
PATH_BOX_BOUNDARY = 0.2

def main():

    # Create tm client
    context = zmq.Context()
    tm_sub = context.socket(zmq.SUB)
    tm_sub.setsockopt_string(zmq.SUBSCRIBE, '')
    tm_sub.connect(TM_SUB_ENDPOINT)

    # Create figure
    fig, ax = plt.subplots()
    plt.ion()
    plt.show(block=False)

    # Error data, which will be added over time
    long_err_data = np.empty((0, 2))
    lat_err_data = np.empty((0, 2))
    head_err_data = np.empty((0, 2))

    path_plot = None
    sec_path_plot = None
    map_plot = None
    target_plot = None
    pose_plot = None

    # Set color maps
    cmap = copy.copy(matplotlib.cm.get_cmap('viridis'))
    cmap.set_bad('white')
    cmap.set_over('red')
    cmap.set_under('white')
         
    while True:

        # Get tm data
        tm = get_tm(tm_sub)
    
        # Draw it on the plot
        if tm is not None:
            time = tm['sim_time_s']
            
            # Clear the axis
            ax.title.set_text(f'Autonomy State (t = {time:.2f})')

            if tm['auto'] is not None:
                if tm['auto']['path'] is not None:
                    # Draw path
                    path = conv_path(tm['auto']['path'])
                    path_plot = plot_path(path, items=path_plot, ax=ax, linespec='-', linecolor='#f926ff')
                if tm['auto']['secondary_path'] is not None:
                    # Draw path
                    path = conv_path(tm['auto']['secondary_path'])
                    sec_path_plot = plot_path(path, items=sec_path_plot, ax=ax, linespec=':', linecolor='#f926ff')
                if tm['auto']['pose'] is not None:
                    # Draw pose
                    r = R.from_quat(tm['auto']['pose']['attitude_q'])
                    forward = r.as_matrix() * np.array([1, 0, 0])
                    forward = forward[0:2, 0]
                    forward = forward / np.linalg.norm(forward)
                    if pose_plot is not None:
                        pose_plot.remove()
                    pose_plot =  ax.quiver(
                        tm['auto']['pose']['position_m'][0],
                        tm['auto']['pose']['position_m'][1],
                        forward[0],
                        forward[1],
                        color='#1dfffe',
                        zorder=2.0
                    )
                if tm['auto']['global_cost_map'] is not None:
                    cm = CellMap.from_raw_dict(tm['auto']['global_cost_map']['map'])
                    for layer in cm.layers:
                        cm.data[layer] = conv_cost_map_data(cm.data[layer])
                    if map_plot is not None:
                        del map_plot
                    map_plot = cm.plot(ax=ax, cmap=cmap, vmin=0.0, vmax=1.0)
        pause(0.00001, False)

num_packets = 0
def get_tm(tm_sub):
    '''
    Get next tm packet from the socket
    '''
    global num_packets

    try: 
        tm_str = tm_sub.recv_string(flags=zmq.NOBLOCK)
        num_packets += 1
        tm = json.loads(tm_str)

        # Only load every 10th packet, or if it has a gcm in it load it
        if num_packets % 10 != 0 and tm['auto']['global_cost_map'] is None:
            tm = None
    except zmq.Again:
        tm = None
    except zmq.ZMQError as e:
        print(f'TmClient: Error - {e}, ({e.errno})')
        tm = None
    except Exception as e:
        print(f'TmClient Exception: {e}')
        tm = None

    return tm

def pause(interval, focus_figure=True):
    backend = matplotlib.rcParams['backend']
    if backend in _interactive_bk:
        figManager = _pylab_helpers.Gcf.get_active()
        if figManager is not None:
            canvas = figManager.canvas
            if canvas.figure.stale:
                canvas.draw()
            if focus_figure:
                plt.show(block=False)
            canvas.start_event_loop(interval)
            return

    # # No on-screen figure is active, so sleep() is all we need.
    # import time
    # time.sleep(interval)

if __name__ == '__main__':
    main()