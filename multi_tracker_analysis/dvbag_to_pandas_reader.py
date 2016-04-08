from optparse import OptionParser
import sys, os

import rosbag, rospy
import pickle

import pandas

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
        else:
            self.dataframe = pandas.DataFrame([[0, 0, 0, 0]], columns=['x','y','value', 'frames'], index=[0])
            self.hdf = pandas.HDFStore(self.saveto)
            self.hdf.put('data', self.dataframe, format='table', data_columns=True)
            
        atexit.register(self.save_pandas_dataframe)
        
    def process_message(self, delta_vid):
        if delta_vid.values is not None:
            if len(delta_vid.values) > 0:
                index = int(delta_vid.header.frame_id)
                indices = [index for i in range(len(delta_vid.values))]
                
                x = pandas.Series(delta_vid.xpixels)
                y = pandas.Series(delta_vid.ypixels)
                value = pandas.Series(delta_vid.values)
                frames = pandas.Series(indices)
                
                df = pandas.DataFrame({'x': x,
                                       'y': y,
                                       'value': value,
                                       'frames': frames})
               
                self.hdf.append('data', df, format='table', data_columns=True)

    def process_timerange(self, timestart, timeend):
        '''
        For realtime processing:
        timestart and timeend determine msg range to read
        '''
        print 'loading image sequence from delta video bag - may take a moment'
        rt0 = rospy.Time(timestart)
        rt1 = rospy.Time(timeend)
        msgs = self.dvbag.read_messages(start_time=rt0, end_time=rt1)
        total_time = timeend - timestart
        realtime = False
        
        pbar = progressbar.ProgressBar().start()
        frame_index_list = []
        for msg in msgs:
            delta_vid = msg[1]
            index = int(delta_vid.header.frame_id)
            frame_index_list.append(index)
            t = delta_vid.header.stamp.secs + delta_vid.header.stamp.nsecs*1e-9
            time_elapsed = t-timestart
            s = int((time_elapsed / total_time)*100)
            if index in self.dataframe.frames.values:
                continue
            
            self.process_message(delta_vid)
            pbar.update(s)
        
        pbar.finish()
        self.dataframe = pandas.read_hdf(self.saveto)
        return frame_index_list
        
    def save_pandas_dataframe(self):
        self.hdf.close() # closes the file
        return
        
#####################################################################################################
    
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--dvbag", type="str", dest="dvbag", default='',
                        help="filename of dvbag file")
    parser.add_option("--saveto", type="str", dest="saveto", default='',
                        help="filename where to save video, default is none")
    
    (options, args) = parser.parse_args()
    
    dvbag2pandasreader = DVBag2PandasReader(options.dvbag, options.saveto)
