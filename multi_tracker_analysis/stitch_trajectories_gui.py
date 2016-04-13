from optparse import OptionParser
import sys, os

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.ptime as ptime

import numpy as np
import time

import read_hdf5_file_to_pandas

import matplotlib.pyplot as plt

import multi_tracker_analysis as mta
import cv2
import copy

import rosbag, rospy

import pandas

import stitch_trajectories
import data_slicing
import read_hdf5_file_to_pandas

import pickle

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
            return os.path.expanduser(os.path.join(path, filename))

def load_data(path):
    try:
        data_filename = get_filename(path, 'trackedobjects.pickle')
        print 'Found file: ', data_filename
        pd = pandas.read_pickle(data_filename)
    except:
        data_filename = get_filename(path, 'trackedobjects.hdf5')
        print 'Found file: ', data_filename
        data_filename_pickled = data_filename.split('.')[0] + '.pickle'
        try:
            pd = pandas.read_pickle(data_filename_pickled)
        except:
            pd = mta.read_hdf5_file_to_pandas.load_data_as_pandas_dataframe_from_hdf5_file(data_filename)
            pandas.to_pickle(pd, data_filename_pickled)
            
    pd = mta.read_hdf5_file_to_pandas.remove_rows_above_speed_threshold(pd, speed_threshold=2)
            
    return pd
            
def load_data_and_delta_video(path):
    pd = load_data(path)
    data_filename = get_filename(path, 'trackedobjects.hdf5')
    delta_video_filename = get_filename(path, 'delta_video.bag')
        
    print 'Loading delta video bag, this may take some time'
    bag = rosbag.Bag(delta_video_filename)
    
    bgimg = load_backgroundimg(path)
    
    return pd, bag, bgimg
    
def load_backgroundimg(path):
    bg_img_filename = get_filename(path, 'deltavideo_bgimg')
    bgimg = cv2.imread(bg_img_filename, cv2.CV_8UC1)
    return bgimg

class QTrajectory(object):
    def __init__(self, path, keys=None):
        self.pd, self.dvbag, self.backgroundimg = load_data_and_delta_video(path)
        self.dataset = read_hdf5_file_to_pandas.Dataset(self.pd)
        self.backgroundimg_filename = get_filename(path, '_bgimg_')
        self.binsx = None
        self.binsy = None
        
        
        self.filename = os.path.expanduser( os.path.join(path, 'stitches.pickle') ) 
        f = open(self.filename, 'w')
        self.stitches = []
        pickle.dump(self.stitches, f)
        f.close()
        
        if keys is None:
            keys = np.unique(self.pd.objid.values)
        self.keys = keys
        self.key_index = -1
        self.key = None
        self.candidate = None
        self.candidates = None
        self.candidate_index = -1
        self.new_key = True
        
        ## create GUI
        self.app = QtGui.QApplication([])
        self.w = QtGui.QWidget()
        self.layout = QtGui.QGridLayout()
        self.w.setLayout(self.layout)
        
        next_btn = QtGui.QPushButton('next match')
        next_btn.pressed.connect(self.next_stitch)
        self.layout.addWidget(next_btn, 0, 0)
        
        save_btn = QtGui.QPushButton('save stitch')
        save_btn.pressed.connect(self.save_stitch)
        self.layout.addWidget(save_btn, 1, 0)
        
        reject_btn = QtGui.QPushButton('reject stitch')
        reject_btn.pressed.connect(self.reject_stitch)
        self.layout.addWidget(reject_btn, 2, 0)
        
        self.button = 'accepted'
        
        self.p1 = pg.PlotWidget(title="Basic array plotting")
        
        # for showing starting point
        self.scatter = pg.ScatterPlotItem()
        self.p1.addItem(self.scatter)
        self.scatter.setZValue(0)
        
        self.layout.addWidget(self.p1, 0, 1)
        
        self.w.show()

        # Enable antialiasing for prettier plots
        #pg.setConfigOptions(antialias=True)
        #self.p1 = self.win.addPlot(title="Basic array plotting")
    
    def next_stitch(self):
        if len(self.stitches) == 0 or self.new_key:
            self.key_index += 1
            key = self.keys[self.key_index] 
        else:
            key = self.stitches[-1][-1]
        print 'Running key: ', key
        
        if self.button == 'accepted':
            self.candidates = stitch_trajectories.get_distance_checked_candidate_keys(self.pd, key, framerange=[2,10], max_distance=20)
            self.candidate_index = 0
            print 'new candidates: ', self.candidates
        else:
            self.candidate_index += 1
            
        if len(self.candidates) >= self.candidate_index:
            candidate = self.candidates[self.candidate_index]
            self.button = 'undecided'
            self.p1.clear()
            self.new_key = False
            self.run(key, candidate)
        else:
            print 'No Candidates!'
            self.button = 'accepted'
            self.new_key = True
            self.next_stitch()
    
    def save_stitch(self):
        f = open(self.filename, 'w')
        self.stitches.append([self.key, self.candidate])
        pickle.dump(self.stitches, f)
        f.close()
        print 'stitch accepted'
        self.button = 'accepted'
        self.next_stitch()
    
    def reject_stitch(self):
        print 'rejected'
        self.button = 'rejected'
        self.next_stitch()
        
    def load_image_sequence(self, timerange):
        rt0 = rospy.Time(timerange[0])
        rt1 = rospy.Time(timerange[1])
        self.msgs = self.dvbag.read_messages(start_time=rt0, end_time=rt1)
        
        self.image_sequence = []
        for msg in self.msgs:
            imgcopy = copy.copy(self.backgroundimg)
            imgcopy[ msg[1].xpixels, msg[1].ypixels] = msg[1].values
            self.image_sequence.append(imgcopy)
        self.current_frame = -1
        
    def get_next_reconstructed_image(self):
        if self.button != 'undecided':
            return None
        if self.current_frame >= len(self.image_sequence)-1:
            self.current_frame = -1
            print 'restarted movie'
        self.current_frame += 1
        return self.image_sequence[self.current_frame]
        
        
    '''        
    def get_next_reconstructed_image(self):
        if self.button != 'undecided':
            return None
        try:
            msg = self.msgs.next()
        except Exception:
            self.load_image_sequence(self.timerange)
            msg = self.msgs.next()
            print 'restarted movie'
        imgcopy = copy.copy(self.backgroundimg)
        imgcopy[ msg[1].xpixels, msg[1].ypixels, 0] = msg[1].values
        imgcopy[ msg[1].xpixels, msg[1].ypixels, 1] = msg[1].values
        imgcopy[ msg[1].xpixels, msg[1].ypixels, 2] = msg[1].values
        return imgcopy
    '''
     
    def play_movie(self, timerange):
        print 'loading image sequence'
        self.timerange = timerange
        self.load_image_sequence(timerange)
    
        print 'playing movie'
        cvimg = copy.copy(self.backgroundimg)#self.get_next_reconstructed_image()
        x = cvimg.shape[0]
        self.img = pg.ImageItem(cvimg)
        self.p1.addItem(self.img)
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
        
    def show_trajectory_pair(self, key1, key2):
        trajec1 = self.dataset.trajec(key1)
        trajec2 = self.dataset.trajec(key2)
        
        self.p1.plot(trajec1.position_y, trajec1.position_x, pen=(0,0,255)) 
        self.p1.plot(trajec2.position_y, trajec2.position_x, pen=(255,0,0)) 
        
        self.p1.plot(trajec1.position_y[0:1], trajec1.position_x[0:1], pen=(0,0,0), symbol='o', symbolSize=10) 
        self.p1.plot(trajec2.position_y[0:1], trajec2.position_x[0:1], pen=(0,0,0), symbol='o', symbolSize=10) 

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

    def go(self):
        ## Start the Qt event loop
        self.app.exec_()
        
    
## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
        
    ## Read data #############################################################
    parser = OptionParser()
    parser.add_option('--path', type=str, help="the .bag file")
    (options, args) = parser.parse_args()

    Qtrajec = QTrajectory(options.path)
    Qtrajec.go()
    
