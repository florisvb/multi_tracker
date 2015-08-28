import os

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import matplotlib.animation as animation
import numpy
import Image
import cv2

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
    
def get_first_and_last_time_points(dataset, keys):
    time_start = np.inf
    time_end = 0
    for key in keys:
        trajec = dataset[key]
        if trajec.time[0] < time_start:
            time_start = trajec.time[0]
        if trajec.time[-1] > time_end:
            time_end = trajec.time[-1]
    return time_start, time_end
    
def fig2img ( fig ):
    #fig.canvas.blit()
    fig.canvas.draw()
    data = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
    data = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    return data
     
class SubplotAnimation(animation.TimedAnimation):        
    def __init__(self, dataset, keys, backgroundimg_filename=None, save_filename=None):    
        self.fig, self.ax = plt.subplots()
        if backgroundimg_filename is not None:
            print 'background image found'
            self.img = plt.imread(backgroundimg_filename)
            self.ax.imshow(self.img, extent=[0,self.img.shape[1],0,self.img.shape[0]], origin='lower',cmap='gray')
            self.ax.set_xlim(0,self.img.shape[1])
            self.ax.set_ylim(0,self.img.shape[0])
        self.ax.set_aspect('equal')
        self.fig.patch.set_visible(False)
        self.ax.axis('off')
        
        ##
        self.save_filename = save_filename
        if save_filename is not None:
            img = fig2img(self.fig)
            self.writer = cv2.VideoWriter(save_filename, cv2.cv.CV_FOURCC('m','p','4','v'), 15, (img.shape[1], img.shape[0]), True)
        else:
            self.writer = None
        ##
        
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
        
        time_start, time_end = get_first_and_last_time_points(dataset, keys)
        self.interval = 1
        n_frames = (time_end - time_start) / float(self.interval)
        self.times = np.linspace(time_start, time_end, n_frames)
            
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
        
        if self.writer is not None:
            if self.writer.isOpened():
                img = fig2img(self.fig)
                #cv2.imwrite('test.png', img)
                #print self.img.shape, img.shape
                self.writer.write(img)
                cv2.waitKey(1)
    
    def _init_draw(self):
        lines =  [line for line in self.keys_to_lines.values()]
        for l in lines:
            l.set_data([], [])
            
    def run_animation(self):
        animation.TimedAnimation.__init__(self, self.fig, interval=50, blit=True)
            
    def save_animation(self):
        for f in range(len(self.times)):
            self._draw_frame(f)
            
if __name__ == '__main__':
    backgroundimg_filename = '/media/LandingPlumeTrak/rotpad/20150322_15sccmco2_vs_air_dot/rotpad/rotpad_20150325/20150325_1759.png'
    keys = keys
    dataset = data
    ani = SubplotAnimation(dataset, keys, backgroundimg_filename=backgroundimg_filename, save_filename='test_sub.avi')
    ani.save_animation()
    #ani.run_animation()
    
    cv2.destroyAllWindows()
    ani.writer.release()
