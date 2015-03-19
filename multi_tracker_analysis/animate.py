import os

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import matplotlib.animation as animation

def get_keys_and_frames_that_for_time_range(dataset, time_start, time_end, keys=None): 
    if keys is None:
        keys = dataset.keys()
    key_and_frame_dict = {}
    for key in keys:
        trajec = dataset[key]
        frames = np.where(np.logical_and(trajec.time>=time_start, trajec.time<=time_end))[0]
        if len(frames) > 0:
            key_and_frame_dict.setdefault(key, frames)
    return key_and_frame_dict
    
def get_first_and_last_time_points(dataset):
    time_start = np.inf
    time_end = 0
    for key, trajec in dataset.items():
        if trajec.time[0] < time_start:
            time_start = trajec.time[0]
        if trajec.time[-1] > time_end:
            time_end = trajec.time[-1]
    return time_start, time_end
            
class SubplotAnimation(animation.TimedAnimation):        
    def __init__(self, dataset, keys, backgroundimg_filename=None):    
        self.fig, self.ax = plt.subplots()
        
        if backgroundimg_filename is not None:
            print 'background image found'
            img = plt.imread(backgroundimg_filename)
            self.ax.imshow(img, extent=[0,img.shape[1],0,img.shape[0]], origin='lower')
            self.ax.set_xlim(0,img.shape[1])
            self.ax.set_ylim(0,img.shape[0])
        
        self.ax.set_aspect('equal')
        
        self.dataset = dataset
        self.keys = []
        for key in keys:
            trajec = dataset[key]
            if trajec.length > 5:
                self.keys.append(key)
        
        self.keys_to_lines = {}
        for key in self.keys:
            trajec = dataset[key]
            line = matplotlib.lines.Line2D([], [], color='blue')#ax.plot(trajec.position[:,0], trajec.position[:,1])
            self.ax.add_line(line)
            self.keys_to_lines.setdefault(key, line)
        
        time_start, time_end = get_first_and_last_time_points(dataset)
        time_start = 1426665000
        self.interval = 3
        n_frames = (time_end - time_start) / float(self.interval)
        self.times = np.linspace(time_start, time_end, n_frames)
            
            
        animation.TimedAnimation.__init__(self, self.fig, interval=2, blit=True)
        
        
    def new_frame_seq(self):
        return iter(range(self.times.size))
    
    def _draw_frame(self, framedata):
        for key, line in self.keys_to_lines.items():
            line.set_data([], [])
            line.set_visible(False)
    
        time_end = self.times[framedata]
        time_start = self.times[framedata]-self.interval*10
        print time_end
        key_and_frame_dict = get_keys_and_frames_that_for_time_range(self.dataset, time_start, time_end, keys=self.keys)
        
        draw_lines = []
        for key, frames in key_and_frame_dict.items():
            line = self.keys_to_lines[key]
            trajec = self.dataset[key]
            line.set_data(trajec.position[frames,0], trajec.position[frames,1])
            line.set_visible(True)
            draw_lines.append(line)
            
        self._drawn_artists = draw_lines
        
    def _init_draw(self):
        lines =  [line for line in self.keys_to_lines.values()]
        for l in lines:
            l.set_data([], [])
            
if __name__ == '__main__':
    backgroundimg_filename = '/media/FlyShapes/rotpad/20150317_15sccmEthanol_vs_air_dot/rotpad/frame0000.jpg'
    keys = keys
    dataset = data
    ani = SubplotAnimation(dataset, keys, backgroundimg_filename=backgroundimg_filename)
    #ani.save('test_sub.mp4')
    plt.show()
