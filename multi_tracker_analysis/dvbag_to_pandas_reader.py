from optparse import OptionParser
import sys, os
import cv2

import numpy as np

import rosbag, rospy
import pickle

import pandas
import threading

import atexit

import progressbar

class DVBag2PandasReader(object):
    def __init__(self, dvbag_filename=None, saveto=None):
        if dvbag_filename is not None:
            self.dvbag = rosbag.Bag(dvbag_filename)
        else:
            print 'Processing from live stream'
        self.saveto = saveto
        
        if os.path.exists(self.saveto):
            self.dataframe = pandas.read_hdf(self.saveto)
            self.hdf = pandas.HDFStore(self.saveto)
            print 'Loaded pre-existing HDF with ' + str(len(self.dataframe)) + ' rows'
        else:
            self.dataframe = pandas.DataFrame([[0, 0, 0, 0, 0, 0, 0]], columns=['x','y','value', 'frames', 'difference', 'time_secs', 'time_nsecs'], index=[0])
            self.hdf = pandas.HDFStore(self.saveto)
            self.hdf.put('data', self.dataframe, format='table', data_columns=True)
          
        first_message = self.dvbag.read_messages().next()[1]
        self.background_image_filename = os.path.basename(first_message.background_image) 
        directory = os.path.dirname(dvbag_filename)
        directory_and_filename = os.path.join(directory, self.background_image_filename)
        self.background_image = cv2.imread(directory_and_filename, 0 )
          
        self.lockBuffer = threading.Lock()
        self.df_buffer = None
            
        atexit.register(self.save_pandas_dataframe)
        
    def process_message(self, delta_vid):
        if delta_vid.values is not None:
            if len(delta_vid.values) > 0:
                index = int(delta_vid.header.frame_id)
                indices = [index for i in range(len(delta_vid.values))]
                secs = [delta_vid.header.stamp.secs for i in range(len(delta_vid.values))]
                nsecs = [delta_vid.header.stamp.nsecs for i in range(len(delta_vid.values))]
                
                if os.path.basename(delta_vid.background_image) != self.background_image_filename:
                    print 'Warning: not using new background image!'
                
                x = pandas.Series(delta_vid.xpixels)
                y = pandas.Series(delta_vid.ypixels)
                value = pandas.Series(delta_vid.values)
                frames = pandas.Series(indices)
                
                df = pandas.DataFrame({'x': x,
                                       'y': y,
                                       'value': value,
                                       'difference': value - self.background_image[x,y],
                                       'frames': frames,
                                       'time_secs': secs,
                                       'time_nsecs': nsecs})
                print df    
                if self.df_buffer is None:
                    self.df_buffer = df
                else:
                    self.df_buffer = self.df_buffer.append(df)
                    
        if len(self.df_buffer) > 50000:
            self.dump_buffer_to_disk()
                
    def dump_buffer_to_disk(self): 
        with self.lockBuffer:
            self.hdf.append('data', self.df_buffer, format='table', data_columns=True)
            self.df_buffer = None

    def process_timerange(self, timestart, timeend):
        '''
        For realtime processing:
        timestart and timeend determine msg range to read. They must be integers. 
        '''
        timestart = int(timestart)
        timeend = int(timeend)
        rt0 = rospy.Time(timestart)
        rt1 = rospy.Time(timeend)
        msgs = self.dvbag.read_messages(start_time=rt0, end_time=rt1)
        total_time = timeend - timestart
        realtime = False
        
        pbar = progressbar.ProgressBar().start()
        if timestart in self.dataframe.time_secs.values and timeend in self.dataframe.time_secs.values:
            q = 'time_secs == ' + str(timestart)
            framestart = np.min(self.dataframe.query(q).frames)
            q = 'time_secs == ' + str(timeend)
            frameend = np.max(self.dataframe.query(q).frames)
            frame_index_list = np.arange(framestart, frameend).tolist()
            print 'Loading already processed data'
        else:
            print 'loading image sequence from delta video bag - may take a moment'
            pbar = progressbar.ProgressBar().start()
            frame_index_list = []
            for msg in msgs:
                delta_vid = msg[1]
                index = int(delta_vid.header.frame_id)
                frame_index_list.append(index)
                t = delta_vid.header.stamp.secs + delta_vid.header.stamp.nsecs*1e-9
                time_elapsed = t-timestart
                s = int((time_elapsed / total_time)*100)
                pbar.update(s)
                if index in self.dataframe.frames.values:
                    continue
                
                self.process_message(delta_vid)
        
        pbar.finish()
        
        self.save_pandas_dataframe()
        self.dataframe = pandas.read_hdf(self.saveto)
        return frame_index_list, self.dataframe
        
    def save_pandas_dataframe(self):
        self.dump_buffer_to_disk()
        self.hdf.close() # closes the file
        return
        
def plot_heatmap_of_dvbag(dvbag_pandas, threshold, bgimg_filename=None):

    if bgimg_filename is not None:
        background_image = cv2.imread(bgimg_filename, 0 )
        shape = background_image.shape
    else:
        shape = (600,800)
        
    binsx = np.arange(0,shape[0],5)
    binsy = np.arange(0,shape[1],5)
        
        
    
        
    dvbag_pandas_subset = dvbag_pandas.query('difference < -30')
    hist, binsx, binsy = np.histogram2d(dvbag_pandas_subset.x.values, dvbag_pandas_subset.y.values, (binsx, binsy))
    
        
#####################################################################################################
    
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--dvbag", type="str", dest="dvbag", default='',
                        help="filename of dvbag file")
    parser.add_option("--saveto", type="str", dest="saveto", default='',
                        help="filename where to save video, default is none")
    
    (options, args) = parser.parse_args()
    
    dvbag2pandasreader = DVBag2PandasReader(options.dvbag, options.saveto)
