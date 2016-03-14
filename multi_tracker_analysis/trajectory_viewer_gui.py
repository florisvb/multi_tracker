from optparse import OptionParser
import sys, os

import rosbag, rospy

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

class QTrajectory(object):
    def __init__(self, pd, bgimg, delta_video_filename):
        self.pd = pd
        self.dataset = read_hdf5_file_to_pandas.Dataset(self.pd)
        self.backgroundimg_filename = bgimg
        self.backgroundimg = None
        self.binsx = None
        self.binsy = None
        self.troi = [np.min(pd.time_epoch.values), np.min(pd.time_epoch.values)+300] 
        
        # load delta video bag
        if delta_video_filename != 'none':
            print 'Loading delta video bag, this may take some time'
            self.dvbag = rosbag.Bag(delta_video_filename)
        else:
            self.dvbag = None
        
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
        
        
        self.p1 = pg.PlotWidget(title="Basic array plotting", x=self.time_epoch_continuous, y=self.nflies)
        self.p1.enableAutoRange('xy', False)
        self.layout.addWidget(self.p1, 1, 1)
        
        self.p2 = pg.PlotWidget()
        self.layout.addWidget(self.p2, 0, 1)
        
        lr = pg.LinearRegionItem(values=self.troi)
        f = 'update_time_region'
        lr.sigRegionChanged.connect(self.__getattribute__(f))
        self.p1.addItem(lr)
        
    def update_time_region(self, linear_region):
        self.p2.clear()
        
        self.troi = linear_region.getRegion()
        pd_subset = mta.data_slicing.get_data_in_epoch_timerange(self.pd, self.troi)
        
        if self.binsx is None:
            self.binsx, self.binsy = mta.plot.get_bins_from_backgroundimage(self.backgroundimg_filename)
            self.backgroundimg = cv2.imread(self.backgroundimg_filename)
        img = copy.copy(self.backgroundimg)
        
        h = mta.plot.get_heatmap(pd_subset, self.binsy, self.binsx, position_x='position_y', position_y='position_x', position_z='position_z', position_z_slice=None)
        
        indices = np.where(h != 0)
        #masked_heatmap_binary = np.ma.masked_array(heatmap_binary, mask=np.invert(heatmap_binary))
    
        print np.max(self.backgroundimg), np.max(h)
        print 
        img[indices] = 0
        
        self.img = pg.ImageItem(img)
        self.p2.addItem(self.img)
        self.img.setZValue(-200)  # make sure image is behind other data
        #self.p2.setImage(img.swapaxes(0,1)[:,:,[2,1,0]], autoRange=False, autoLevels=False, )
        
        keys = np.unique(pd_subset.objid.values)
        if len(keys) < 20:
            for key in keys:
                trajec = self.dataset.trajec(key)
                self.p2.plot(trajec.position_y, trajec.position_x, pen=(255,0,0)) 
        
        print 'region: ', self.troi
        print mta.read_hdf5_file_to_pandas.get_objid_lengths(pd_subset)
        print
        print pd.speed.groupby(pd.objid).max()
        
    ### Delta video bag stuff
    def load_image_sequence(self):
        timerange = self.troi
        print 'loading image sequence from delta video bag - may take a moment'
        rt0 = rospy.Time(timerange[0])
        rt1 = rospy.Time(timerange[1])
        self.msgs = self.dvbag.read_messages(start_time=rt0, end_time=rt1)
        
        self.image_sequence = []
        for msg in self.msgs:
            imgcopy = copy.copy(self.backgroundimg)
            try:
                imgcopy[ msg[1].xpixels, msg[1].ypixels, 0] = msg[1].values
                imgcopy[ msg[1].xpixels, msg[1].ypixels, 1] = msg[1].values
                imgcopy[ msg[1].xpixels, msg[1].ypixels, 2] = msg[1].values
            except:
                print 'bad message?'
            self.image_sequence.append(imgcopy)
        self.current_frame = -1
    
    def get_next_reconstructed_image(self):
        if self.current_frame >= len(self.image_sequence)-1:
            self.current_frame = -1
            print 'restarted movie'
        self.current_frame += 1
        print self.current_frame, len(self.image_sequence)
        return self.image_sequence[self.current_frame]
    
    def play_movie(self):
        timerange = self.troi
        print 'loading image sequence'
        self.load_image_sequence()
    
        print 'playing movie'
        cvimg = copy.copy(self.backgroundimg)#self.get_next_reconstructed_image()
        x = cvimg.shape[0]
        self.img = pg.ImageItem(cvimg)
        self.p2.addItem(self.img)
        self.img.setZValue(-100)  # make sure image is behind other data
        #img.setRect(pg.QtCore.QRectF(0, 0, 4, 5))

        self.updateTime = ptime.time()
        self.fps = 0
        
        self.updateData()
    
    def updateData(self):
        #global img, data, i, updateTime, fps

        ## Display the data
        cvimg = self.get_next_reconstructed_image()
        self.img.setImage(cvimg)
        
        QtCore.QTimer.singleShot(1, self.updateData)
        now = ptime.time()
        dt = (now-self.updateTime)
        self.updateTime = now
        #self.fps = self.fps * 0.9 + fps2 * 0.1
        
        if dt < 0.02:
            d = 0.02 - dt
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
        
        print t0, t1
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
    parser.add_option('--filename', type=str, help="name and path of the hdf5 tracked_objects filename")
    parser.add_option('--bgimg', type=str, help="name and path of the background image")
    parser.add_option('--minlength', type=int, default=1, help="minimum length of trajectories to show")
    parser.add_option('--minspeed', type=float, default=0, help="minimum length of trajectories to show")
    parser.add_option('--maxspeed', type=float, default=10, help="maximum speed")
    parser.add_option('--dvbag', type=str, default='none', help="name and path of the delta video bag file, optional")
    (options, args) = parser.parse_args()

    pd = mta.read_hdf5_file_to_pandas.load_data_as_pandas_dataframe_from_hdf5_file(options.filename)
    pd = mta.read_hdf5_file_to_pandas.cull_short_trajectories(pd, options.minlength)
    pd = mta.read_hdf5_file_to_pandas.remove_rows_above_speed_threshold(pd, speed_threshold=options.maxspeed)
    pd = mta.read_hdf5_file_to_pandas.remove_objects_that_never_exceed_minimum_speed(pd, speed_threshold=options.minspeed)
    
    Qtrajec = QTrajectory(pd, options.bgimg, options.dvbag)
    Qtrajec.run()
