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
    fig = plt.figure()
    ax = fig.gca()
    plt.ion()
    plt.show(block=False)
         
    while True:

        # Get tm data
        tm = get_tm(tm_sub)
    
        # Draw it on the plot
        if tm is not None:
            # Clear the axis
            ax.clear()
            plt.title(f'Autonomy State (t = {tm["sim_time_s"]:.2f})')

            if tm['auto'] is not None:
                if tm['auto']['path'] is not None:
                    # Draw path
                    path = conv_path(tm['auto']['path'])
                    plot_path(path, ax)

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
                    ax.set_xlim(
                        path_centre_x - path_semi_range - PATH_BOX_BOUNDARY,
                        path_centre_x + path_semi_range + PATH_BOX_BOUNDARY
                    )
                    ax.set_ylim(
                        path_centre_y - path_semi_range - PATH_BOX_BOUNDARY,
                        path_centre_y + path_semi_range + PATH_BOX_BOUNDARY
                    )

                    if tm['auto']['traj_ctrl_status'] is not None:
                        target_m = path[tm['auto']['traj_ctrl_status']['target_point_idx'], 0:2]
                        # Highlight the target point
                        ax.plot(target_m[0], target_m[1], 'xb')
                if tm['auto']['pose'] is not None:
                    # Draw pose
                    r = R.from_quat(tm['auto']['pose']['attitude_q_lm'])
                    forward = r.as_matrix() * np.array([1, 0, 0])
                    forward = forward[0:2, 0]
                    forward = forward / np.linalg.norm(forward)
                    ax.quiver(
                        tm['auto']['pose']['position_m_lm'][0],
                        tm['auto']['pose']['position_m_lm'][1],
                        forward[0],
                        forward[1]
                    )
                pause(0.001, False)

    #     def animate(i):
    #     # Clear figure
    #     ax.clear()
    #     ax.set_xlim(-2.0, 2.0)
    #     ax.set_ylim(-2.0, 2.0)

    #     # Get tm data
    #     tm = get_tm(tm_sub)
    
    #     # Draw it on the plot
    #     if tm is not None:
    #         if tm['auto'] is not None:
    #             if tm['auto']['path'] is not None:
    #                 # Draw path
    #                 path = conv_path(tm['auto']['path'])
    #                 plot_path(path, ax)
    #             if tm['auto']['pose'] is not None:
    #                 # Draw pose
    #                 r = R.from_quat(tm['auto']['pose']['attitude_q_lm'])
    #                 forward = r.as_matrix() * np.array([1, 0, 0])
    #                 forward = forward[0, 0:2]
    #                 forward = forward / np.linalg.norm(forward)
    #                 ax.quiver(
    #                     tm['auto']['pose']['position_m_lm'][0],
    #                     tm['auto']['pose']['position_m_lm'][1],
    #                     forward[0],
    #                     forward[1]
    #                 )

    # ani = animation.FuncAnimation(
    #     fig, 
    #     animate,
    #     interval=200,
    #     blit=False
    # )
    # plt.show()

def get_tm(tm_sub):
    '''
    Get next tm packet from the socket
    '''

    try: 
        tm_str = tm_sub.recv_string(flags=zmq.NOBLOCK)
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