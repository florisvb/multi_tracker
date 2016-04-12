from optparse import OptionParser
import sys, os
import imp

import rosbag, rospy
import pickle

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.ptime as ptime
import time
import numpy as np

import orchard.make_orchard_plots as mop
import read_hdf5_file_to_pandas
import data_slicing

import matplotlib.pyplot as plt

import multi_tracker_analysis as mta
import cv2
import copy

import progressbar

def get_filename(path, contains):
    cmd = 'ls ' + path
    ls = os.popen(cmd).read()
    all_filelist = ls.split('\n')
    try:
        all_filelist.remove('')
    except:
        pass
    filelist = []
    for i, filename in enumerate(all_filelist):
        if contains in filename:
            return os.path.join(path, filename)
            
def get_random_color():
    color = (np.random.randint(0,255), np.random.randint(0,255), np.random.randint(0,255))
    return color
    
class QTrajectory(object):
    def __init__(self, pd, bgimg, delta_video_filename, config=None, skip_frames=5):
        '''
        skip_frames - when playing back movie, how many frames to skip between updates (to speed up viewing)
        '''
        self.pd = pd
        self.dataset = read_hdf5_file_to_pandas.Dataset(self.pd)
        self.backgroundimg_filename = bgimg
        self.backgroundimg = None
        self.binsx = None
        self.binsy = None
        self.troi = [np.min(pd.time_epoch.values), np.min(pd.time_epoch.values)+300] 
        self.config = config
        self.skip_frames = skip_frames
        
        # load delta video bag
        if delta_video_filename != 'none':
            self.dvbag = rosbag.Bag(delta_video_filename)
        else:
            self.dvbag = None
            
        # trajectory colors
        self.trajec_to_color_dict = {}
        
        # extract time and speed
        self.time_epoch = pd.time_epoch.groupby(pd.index).mean().values
        self.speed = pd.speed.groupby(pd.index).mean().values
        self.nflies = data_slicing.get_nkeys_per_frame(pd)
        self.time_epoch_continuous = np.linspace(np.min(self.time_epoch), np.max(self.time_epoch), len(self.nflies))
        
        ## create GUI
        self.app = QtGui.QApplication([])
        self.w = QtGui.QWidget()
        self.layout = QtGui.QGridLayout()
        self.w.setLayout(self.layout)
        self.w.show()

        # play movie button        
        play_btn = QtGui.QPushButton('play video sequence')
        play_btn.pressed.connect(self.play_movie)
        self.layout.addWidget(play_btn, 0, 0)
        
        # stop movie button        
        stop_btn = QtGui.QPushButton('stop video sequence')
        stop_btn.pressed.connect(self.stop_movie)
        self.layout.addWidget(stop_btn, 1, 0)
        
        # save timerange button       
        save_btn = QtGui.QPushButton('save time sequence')
        save_btn.pressed.connect(self.save_time_sequence)
        self.layout.addWidget(save_btn, 2, 0)
        
        
        self.p1 = pg.PlotWidget(title="Basic array plotting", x=self.time_epoch_continuous, y=self.nflies)
        self.p1.enableAutoRange('xy', False)
        self.layout.addWidget(self.p1, 1, 1)
        if self.config is not None:
            print '**** Sensory stimulus: ', self.config.sensory_stimulus_on
            for r, row in enumerate(self.config.sensory_stimulus_on):
                v1 = pg.PlotDataItem([self.config.sensory_stimulus_on[r][0],self.config.sensory_stimulus_on[r][0]], [0,10])
                v2 = pg.PlotDataItem([self.config.sensory_stimulus_on[r][-1],self.config.sensory_stimulus_on[r][-1]], [0,10])
                f12 = pg.FillBetweenItem(curve1=v1, curve2=v2, brush=pg.mkBrush('r'))
                self.p1.addItem(f12)
        
        self.p2 = pg.PlotWidget()
        self.layout.addWidget(self.p2, 0, 1)
        
        lr = pg.LinearRegionItem(values=self.troi)
        f = 'update_time_region'
        lr.sigRegionChanged.connect(self.__getattribute__(f))
        self.p1.addItem(lr)
        
    def save_time_sequence(self):
        self.troi
        filename = os.path.join(config.path, 'time_ranges.pickle')
        if os.path.exists(filename):
            f = open(filename, 'r+')
            time_range_data = pickle.load(f)
            f.close()
        else:
            f = open(filename, 'w+')
            f.close()
            time_range_data = []
        t = self.troi[-1] - self.troi[0]
        data_to_save = [self.troi[0], t]
        time_range_data.append(data_to_save)
        f = open(filename, 'r+')
        pickle.dump(time_range_data, f)
        f.close()
        print 'saved: ', data_to_save
        
    def update_time_region(self, linear_region):
        self.p2.clear()
        
        self.troi = linear_region.getRegion()
        pd_subset = mta.data_slicing.get_data_in_epoch_timerange(self.pd, self.troi)
        
        if self.binsx is None:
            self.binsx, self.binsy = mta.plot.get_bins_from_backgroundimage(self.backgroundimg_filename)
            self.backgroundimg = cv2.imread(self.backgroundimg_filename, cv2.CV_8UC1)
        img = copy.copy(self.backgroundimg)
        
        h = mta.plot.get_heatmap(pd_subset, self.binsy, self.binsx, position_x='position_y', position_y='position_x', position_z='position_z', position_z_slice=None)
        
        indices = np.where(h != 0)
        #masked_heatmap_binary = np.ma.masked_array(heatmap_binary, mask=np.invert(heatmap_binary))
        img[indices] = 0
        
        self.img = pg.ImageItem(img)
        self.p2.addItem(self.img)
        self.scatter = pg.ScatterPlotItem()
        self.scatter.pxMode = True
        self.p2.addItem(self.scatter)
        self.scatter.setZValue(0)
        self.img.setZValue(-200)  # make sure image is behind other data
        #self.p2.setImage(img.swapaxes(0,1)[:,:,[2,1,0]], autoRange=False, autoLevels=False, )
        
        keys = np.unique(pd_subset.objid.values)
        if len(keys) < 100:
            for key in keys:
                trajec = self.dataset.trajec(key)
                first_time = np.max([self.troi[0], trajec.time_epoch[0]])
                first_time_index = np.argmin( np.abs(trajec.time_epoch-first_time) )
                last_time = np.min([self.troi[-1], trajec.time_epoch[-1]])
                last_time_index = np.argmin( np.abs(trajec.time_epoch-last_time) )
                #if trajec.length > 5:
                if key not in self.trajec_to_color_dict.keys():
                    color = get_random_color()
                    self.trajec_to_color_dict.setdefault(key, color)
                else:
                    color = self.trajec_to_color_dict[key]
                pen = pg.mkPen(color, width=2)  
                self.p2.plot(trajec.position_y[first_time_index:last_time_index], trajec.position_x[first_time_index:last_time_index], pen=pen) 
        
    ### Delta video bag stuff
    
    def load_image_sequence(self):
        timerange = self.troi
        print 'loading image sequence from delta video bag - may take a moment'
        pbar = progressbar.ProgressBar().start()
        
        rt0 = rospy.Time(timerange[0])
        rt1 = rospy.Time(timerange[1])
        self.msgs = self.dvbag.read_messages(start_time=rt0, end_time=rt1)
        
        self.image_sequence = []
        for m, msg in enumerate(self.msgs):
            imgcopy = copy.copy(self.backgroundimg)
            imgcopy[ msg[1].xpixels, msg[1].ypixels] = msg[1].values # if there's an error, check if you're using ROS hydro?
            self.image_sequence.append(imgcopy)
            #s = int((m / float(len(self.msgs)))*100)
            tfloat = msg[1].header.stamp.secs + msg[1].header.stamp.nsecs*1e-9
            t_elapsed = tfloat - timerange[0]
            t_total = timerange[1] - timerange[0]
            s = int(100*(t_elapsed / t_total))
            pbar.update(s)
        pbar.finish()
        self.current_frame = -1
        
    def get_next_reconstructed_image(self):
        if self.current_frame >= len(self.image_sequence)-1:
            self.current_frame = -1
            print 'restarted movie'
        self.current_frame += 1
        img = self.image_sequence[self.current_frame]      
        return img
    
    def stop_movie(self):
        self.play = False
    
    def play_movie(self):
        self.play = True
        timerange = self.troi
        print 'loading image sequence'
        self.load_image_sequence()
    
        print 'playing movie'
        self.updateTime = ptime.time()
        self.updateData()
    
    def updateData(self):
        if self.play:
            ## Display the data
            cvimg = self.get_next_reconstructed_image()
            self.img.setImage(cvimg)
            
            QtCore.QTimer.singleShot(1, self.updateData)
            now = ptime.time()
            dt = (now-self.updateTime)
            self.updateTime = now
            
            if dt < 0.005:
                d = 0.005 - dt
                time.sleep(d)
            
            del(cvimg)
    
    def run(self, key, candidate):
        self.key = key
        self.candidate = candidate
        
        self.button = 'undecided'
    
        self.show_trajectory_pair(key, candidate)
        
        frame = np.max(self.pd[self.pd.objid==key].frames)
        pd_subset = data_slicing.get_data_in_framerange(self.pd, [frame-100, frame+100])
        
        t0 = np.min(pd_subset.time_epoch.values)
        t1 = np.max(pd_subset.time_epoch.values)
        
        self.play_movie([t0, t1])

    ###########################

    def run(self):

        ## Display the widget as a new window
        self.w.show()

        ## Start the Qt event loop
        self.app.exec_()
        
## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
        
    ## Read data #############################################################
    parser = OptionParser()
    parser.add_option('--path', type=str, default='none', help="option: path that points to standard named filename, background image, dvbag, config. If using 'path', no need to provide filename, bgimg, dvbag, and config. Note")
    parser.add_option('--movie', type=int, default=1, help="load and play the dvbag movie, default is 1, to load use 1")
    parser.add_option('--filename', type=str, help="name and path of the hdf5 tracked_objects filename")
    parser.add_option('--bgimg', type=str, help="name and path of the background image")
    parser.add_option('--minlength', type=int, default=1, help="minimum length of trajectories to show")
    parser.add_option('--minspeed', type=float, default=0, help="minimum length of trajectories to show")
    parser.add_option('--maxspeed', type=float, default=10, help="maximum speed")
    parser.add_option('--skip-frames', type=int, default=8, dest="skip_frames", help="how many frames to skip between image updates (speeds up processing)")
    parser.add_option('--dvbag', type=str, default='none', help="name and path of the delta video bag file, optional")
    parser.add_option('--config', type=str, default='none', help="name and path of a configuration file, optional. If the configuration file has an attribute 'sensory_stimulus_on', which should be a list of epoch timestamps e.g. [[t1,t2],[t3,4]], then these timeframes will be highlighted in the gui.")
    (options, args) = parser.parse_args()
    
    if options.path != 'none':
        options.filename = get_filename(options.path, 'trackedobjects.hdf5')
        options.config = get_filename(options.path, 'config')
        options.dvbag = get_filename(options.path, 'delta_video.bag')
        options.bgimg = get_filename(options.path, '_bgimg_')
            
    pd = mta.read_hdf5_file_to_pandas.load_data_as_pandas_dataframe_from_hdf5_file(options.filename)
    pd = mta.read_hdf5_file_to_pandas.cull_short_trajectories(pd, options.minlength)
    pd = mta.read_hdf5_file_to_pandas.remove_rows_above_speed_threshold(pd, speed_threshold=options.maxspeed)
    pd = mta.read_hdf5_file_to_pandas.remove_objects_that_never_exceed_minimum_speed(pd, speed_threshold=options.minspeed)
    
    if options.config != 'none':
        Config = imp.load_source('Config', options.config)
        config = Config.Config(os.path.dirname(options.config))
    else:
        config = None
        
    if options.movie is False:
        options.dvbag = 'none'
    
    Qtrajec = QTrajectory(pd, options.bgimg, options.dvbag, config, options.skip_frames)
    Qtrajec.run()
