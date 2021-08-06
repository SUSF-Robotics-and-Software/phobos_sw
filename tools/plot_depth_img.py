'''
Plots a depth image from the given JSON data
'''

import argparse
from pathlib import Path
import matplotlib.pyplot as plt
import json
import numpy as np
import time
import os
from watchdog.observers import Observer
from watchdog.events import PatternMatchingEventHandler

class DepthImgEventHandler(PatternMatchingEventHandler):
    # DEPTH_IMGS_PATTERNS =
    # [r'^\.\./sessions/rov_exec_\d{8}_\d{6}/depth_imgs/depth_img_\d{8}_\d{6}\.json$']
    DEPTH_IMGS_PATTERNS = ["depth_img_*.json"]

    ax = None
    imshow = None
    
    def __init__(self):
        super().__init__(self.DEPTH_IMGS_PATTERNS)
        _, self.ax = plt.subplots()
        plt.ion()
        plt.show()

        # Plot existing images
        ret = plot_latest_img(None, ax=self.ax)
        if ret is None:
            self.ax.set_title('No images yet...')
        else:
            self.ax = ret[0]
            self.imshow = ret[1]

    def on_created(self, event):
        img = load_depth_img(Path(event.src_path))

        if self.imshow is None:
            self.ax, self.imshow = plot_depth_img(img, ax=self.ax)
        else:
            self.ax, self.imshow = plot_depth_img(img, ax=self.ax, imshow=self.imshow)
        
def main(args):
    '''
    Runs the image plotter
    '''

    if args.live:
        live_plot()
        exit()

    if args.path is None:
        session_dir = Path(__file__).parent.resolve().joinpath('../sessions')
        sessions = list(session_dir.glob('rov_exec*'))
        if len(sessions) == 0:
            raise RuntimeError('Could not find any session directories')
        latest_session = max(sessions, key=lambda p: p.stat().st_ctime)
        path = Path(latest_session).joinpath("depth_imgs")
    else:
        path = Path(args.path)

    # Check if the path is a dir or image
    if path.is_dir() and not args.live:
        paths = list(path.glob('depth_img_*.json'))
        if len(paths) == 0:
            raise RuntimeError('Session contains no depth images')
        latest_path = max(paths, key=lambda p: p.stat().st_ctime)
        img = load_depth_img(latest_path)
    elif path.exists() and path.is_file():
        img = load_depth_img(path)
    else:
        live_plot(path)

    plot_depth_img(img)
    plt.show()

def plot_latest_img(path, ax=None, imshow=None):
    '''
    Plots the latest image from the given path
    '''

    if path is None:
        session_dir = Path(__file__).parent.resolve().joinpath('../sessions')
        sessions = list(session_dir.glob('rov_exec*'))
        if len(sessions) == 0:
            raise RuntimeError('Could not find any session directories')
        latest_session = max(sessions, key=lambda p: p.stat().st_ctime)
        path = Path(latest_session).joinpath("depth_imgs")
    else:
        path = Path(path)

    if not path.exists():
        return None
    
    paths = list(path.glob('depth_img_*.json'))
    if len(paths) == 0:
        return None
    latest_path = max(paths, key=lambda p: p.stat().st_ctime)
    img = load_depth_img(latest_path)
    ax, imshow = plot_depth_img(img, ax, imshow)
    return (ax, imshow)

def live_plot():
    '''
    Shows a live updating plot of images in the given directory
    '''
    path = Path(__file__).parent.joinpath('../sessions')

    event_handler = DepthImgEventHandler()
    observer = Observer()
    observer.schedule(event_handler, path, recursive=True)
    observer.start()

    try:
        while True:
            plt.gcf().canvas.draw_idle()
            plt.gcf().canvas.start_event_loop(0.1)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()

def plot_depth_img(img, ax=None, imshow=None):
    '''
    Plots the given depth image.
    '''

    if ax is None:
        fig, ax = plt.subplots()

    if imshow is None:
        imshow = ax.imshow(img['data'], cmap='gray', vmin=img['min'], vmax=img['max'])
    else:
        imshow.set_data(img['data'])
        imshow.set_clim(vmin=img['min'], vmax=img['max'])
        plt.draw()

    ax.set_title(img['name'])

    return (ax, imshow)

def load_depth_img(path: Path):
    '''
    Loads a depth image from the given path
    '''
    # Wait until the file is unlocked
    while not os.access(path, os.W_OK):
        pass

    # Might not actually be able to read the file if it hasn't been finished
    # writing, so we loop here
    num_fails = 0
    while True:
        try:
            with open(path, 'r') as f:
                raw = json.load(f)
                break
        except json.decoder.JSONDecodeError as e:
            if num_fails < 1000:
                num_fails += 1
            else:
                print(f'Tried {num_fails} times to read image, but got error')
                raise e

    img = {
        'timestamp': raw['timestamp'],
        'path': path,
        'name': path.name,
        **raw['image']
    }
    img['data'] = np.array(img['data']).reshape((img['height'], img['width']))
    img['min'] = np.min(img['data'])
    img['max'] = np.max(img['data'])

    return img

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'path',
        nargs='?',
        help='Path to the depth image, or to the directory containing depth images. If a directory the most recent image will be plotted'
    )
    parser.add_argument(
        '--live',
        action='store_true',
        help='If set the script will monitor the path and will update the plot if a more recent image is saved there'
    )

    main(parser.parse_args())