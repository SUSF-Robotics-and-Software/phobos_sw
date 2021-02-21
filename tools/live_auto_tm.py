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
    fig, axs = plt.subplots(2, 1, gridspec_kw={'height_ratios': [3, 1]})
    ax_path = axs[0]
    ax_errors = axs[1]
    plt.ion()
    plt.show(block=False)

    # Error data, which will be added over time
    long_err_data = np.empty((0, 2))
    lat_err_data = np.empty((0, 2))
    head_err_data = np.empty((0, 2))
         
    while True:

        # Get tm data
        tm = get_tm(tm_sub)
    
        # Draw it on the plot
        if tm is not None:
            time = tm['sim_time_s']
            
            # Clear the axis
            ax_path.clear()
            ax_path.title.set_text(f'Autonomy State (t = {time:.2f})')

            if tm['auto'] is not None:
                if tm['auto']['path'] is not None:
                    # Draw path
                    path = conv_path(tm['auto']['path'])
                    plot_path(path, ax_path)

                    # Get the min/max of the path x and y
                    path_min_x = np.min(path[:,0])
                    path_max_x = np.max(path[:,0])
                    path_min_y = np.min(path[:,1])
                    path_max_y = np.max(path[:,1])
                    path_centre_x = (path_min_x + path_max_x) / 2
                    path_centre_y = (path_min_y + path_max_y) / 2

                    # Want a square box around the path, so choose the largest
                    # range to use
                    path_semi_range = max(
                        (path_max_x - path_min_x)/2, 
                        (path_max_y - path_min_y)/2,
                        MIN_PLOT_BOX_SIZE/2
                    )

                    # Set the limits
                    ax_path.set_xlim(
                        path_centre_x - path_semi_range - PATH_BOX_BOUNDARY,
                        path_centre_x + path_semi_range + PATH_BOX_BOUNDARY
                    )
                    ax_path.set_ylim(
                        path_centre_y - path_semi_range - PATH_BOX_BOUNDARY,
                        path_centre_y + path_semi_range + PATH_BOX_BOUNDARY
                    )

                    if tm['auto']['traj_ctrl_status'] is not None:
                        target_m = path[tm['auto']['traj_ctrl_status']['target_point_idx'], 0:2]
                        # Highlight the target point
                        ax_path.plot(target_m[0], target_m[1], 'xb')
                if tm['auto']['pose'] is not None:
                    # Draw pose
                    r = R.from_quat(tm['auto']['pose']['attitude_q_lm'])
                    forward = r.as_matrix() * np.array([1, 0, 0])
                    forward = forward[0:2, 0]
                    forward = forward / np.linalg.norm(forward)
                    ax_path.quiver(
                        tm['auto']['pose']['position_m_lm'][0],
                        tm['auto']['pose']['position_m_lm'][1],
                        forward[0],
                        forward[1]
                    )
                if tm['auto']['traj_ctrl_status'] is not None:
                    lat_err_data = np.append(
                        lat_err_data, 
                        np.array([[time, tm['auto']['traj_ctrl_status']['lat_error_m']]]), axis=0
                    )
                    long_err_data = np.append(
                        long_err_data, 
                        np.array([[time, tm['auto']['traj_ctrl_status']['long_error_m']]]), axis=0
                    )
                    head_err_data = np.append(
                        head_err_data, 
                        np.array([[time, tm['auto']['traj_ctrl_status']['head_error_rad']]]), axis=0
                    )
                    
                    ax_errors.clear()
                    ax_errors.axhline()
                    ax_errors.set_xlabel('Time [s]')
                    ax_errors.plot(lat_err_data[:,0], lat_err_data[:,1], '-r')
                    ax_errors.plot(long_err_data[:,0], long_err_data[:,1], '-b')
                    ax_errors.plot(head_err_data[:,0], head_err_data[:,1], '-g')
                pause(0.00001, False)

def get_tm(tm_sub):
    '''
    Get next tm packet from the socket
    '''

    try: 
        tm_str = tm_sub.recv_string()
        tm = json.loads(tm_str)
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