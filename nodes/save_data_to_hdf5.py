#!/usr/bin/env python
from __future__ import division
from optparse import OptionParser
import roslib
import rospy
import os
import time
import threading

import numpy as np
from multi_tracker.msg import Trackedobject, Trackedobjectlist

import h5py

import atexit

class DataListener:
    def __init__(self, nodenum, info='data information', record_time_hrs=24):
        self.subTrackedObjects = rospy.Subscriber('multi_tracker/' + nodenum + '/tracked_objects', Trackedobjectlist, self.tracked_object_callback, queue_size=300)
        
        experiment_basename = rospy.get_param('/multi_tracker/' + nodenum + '/experiment_basename', 'none')
        if experiment_basename == 'none':
            experiment_basename = time.strftime("%Y%m%d_%H%M%S_N" + nodenum, time.localtime())
           
        filename = experiment_basename + '_trackedobjects.hdf5'
        home_directory = os.path.expanduser( rospy.get_param('/multi_tracker/' + nodenum + '/data_directory') )
        filename = os.path.join(home_directory, filename)
        print 'Saving hdf5 data to: ', filename
        self.time_start = time.time()
        self.record_time_hrs = record_time_hrs
        
        self.buffer = []
        self.array_buffer = []
        # set up thread locks
        self.lockParams = threading.Lock()
        self.lockBuffer = threading.Lock()
        
        self.chunk_size = 5000
        self.hdf5 = h5py.File(filename, 'w')
        #self.hdf5.swmr_mode = True # helps prevent file corruption if closed improperly
        self.hdf5.attrs.create("info", info)
        
        self.data_to_save = [   'objid',
                                'header.stamp.secs',
                                'header.stamp.nsecs', 
                                'header.frame_id', 
                                'position.x', 
                                'position.y', 
                                'position.z', 
                                'velocity.x',
                                'velocity.y',
                                'velocity.z',
                                'angle',
                                'size',
                                'covariance',
                                'measurement.x',
                                'measurement.y',
                                ]
        self.data_format = {    'objid': int,
                                'header.stamp.secs': int,
                                'header.stamp.nsecs': int, 
                                'header.frame_id': int, 
                                'position.x': float, 
                                'position.y': float, 
                                'position.z': float, 
                                'velocity.x': float,
                                'velocity.y': float,
                                'velocity.z': float,
                                'angle': float,
                                'size': float,
                                'covariance': float,
                                'measurement.x': float,
                                'measurement.y': float,
                            }
                            
        self.dtype = [(data,self.data_format[data]) for data in self.data_to_save]
        rospy.init_node('save_data_to_hdf5_' + nodenum)
        
        self.hdf5.create_dataset('data', (self.chunk_size, 1), maxshape=(None,1), dtype=self.dtype)
        self.hdf5['data'].attrs.create('current_frame', 0)
        self.hdf5['data'].attrs.create('line', 0)
        self.hdf5['data'].attrs.create('length', self.chunk_size)
        
    def add_chunk(self):
        length = self.hdf5['data'].attrs.get('length')
        new_length = length + self.chunk_size
        self.hdf5['data'].resize(new_length, axis=0)
        self.hdf5['data'].attrs.modify('length', new_length)
            
    def save_array_data(self):
        newline = self.hdf5['data'].attrs.get('line') + 1
        nrows_to_add = len(self.array_buffer)
        
        self.hdf5['data'].attrs.modify('line', newline+nrows_to_add)
        if newline+nrows_to_add >= self.hdf5['data'].attrs.get('length')-50:
            self.hdf5.flush()
            self.add_chunk()
        
        self.hdf5['data'][newline:newline+nrows_to_add] = self.array_buffer
        self.array_buffer = []
                                                   
    def tracked_object_callback(self, tracked_objects):
        with self.lockBuffer:
            for tracked_object in tracked_objects.tracked_objects:
                a = np.array([(     tracked_object.objid,
                                    tracked_object.header.stamp.secs,
                                    tracked_object.header.stamp.nsecs,
                                    tracked_object.header.frame_id,
                                    tracked_object.position.x, tracked_object.position.y, tracked_object.position.z,
                                    tracked_object.velocity.x, tracked_object.velocity.y, tracked_object.velocity.z,
                                    tracked_object.angle,
                                    tracked_object.size,
                                    tracked_object.covariance,
                                    tracked_object.measurement.x, tracked_object.measurement.y,
                               )], dtype=self.dtype)
                self.array_buffer.append(a)
        
    def process_buffer(self):
        self.save_array_data()
            
    def main(self):
        atexit.register(self.stop_saving_data)
        while (not rospy.is_shutdown()):
            t = time.time() - self.time_start
            if t > self.record_time_hrs*3600:
                self.stop_saving_data()
                return
            with self.lockBuffer:
                time_now = rospy.Time.now()
                if len(self.array_buffer) > 0:
                    self.process_buffer()
                pt = (rospy.Time.now()-time_now).to_sec()
                if len(self.buffer) > 9:
                    rospy.logwarn("Data saving processing time exceeds acquisition rate. Processing time: %f, Buffer: %d", pt, len(self.buffer))
            
        
    def stop_saving_data(self):
        self.hdf5.close()
        print 'shut down nicely'
        
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--nodenum", type="str", dest="nodenum", default='1',
                        help="node number, for example, if running multiple tracker instances on one computer")
    parser.add_option("--record-time-hrs", type="int", dest="record_time_hrs", default=24,
                        help="number of hours to record data for")
    (options, args) = parser.parse_args()
    
    datalistener = DataListener(options.nodenum, options.record_time_hrs)
    datalistener.main()
