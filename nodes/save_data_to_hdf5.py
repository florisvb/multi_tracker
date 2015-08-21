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
    def __init__(self, nodenum, info='data information'):
        self.subTrackedObjects = rospy.Subscriber('multi_tracker/' + nodenum + '/tracked_objects', Trackedobjectlist, self.tracked_object_callback, queue_size=300)
        
        filename = rospy.get_param('/multi_tracker/' + nodenum + '/csv_data_filename')
        if filename == 'none':
            filename = time.strftime("%Y%m%d_%H%M_N" + nodenum, time.localtime()) + '.hdf5'
        home_directory = os.path.expanduser( rospy.get_param('/multi_tracker/data_directory') )
        filename = os.path.join(home_directory, filename)
        print 'Saving hdf5 data to: ', filename
        
        self.buffer = []
        # set up thread locks
        self.lockParams = threading.Lock()
        self.lockBuffer = threading.Lock()
        
        self.chunk_size = 10000
        self.hdf5 = h5py.File(filename, 'w')
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
            
    def save_data(self, tracked_object):
        newline = self.hdf5['data'].attrs.get('line') + 1
        self.hdf5['data'].attrs.modify('line', newline)
        if newline >= self.hdf5['data'].attrs.get('length')-50:
                self.add_chunk()
        self.hdf5['data'][newline] = np.array([(     tracked_object.objid,
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
                                                   
    def tracked_object_callback(self, tracked_objects):
        with self.lockBuffer:
            self.buffer.append(tracked_objects)
        
    def process_buffer(self, tracked_objects):
        for tracked_object in tracked_objects.tracked_objects:
            self.save_data(tracked_object)
            
    def main(self):
        atexit.register(self.stop_saving_data)
        while (not rospy.is_shutdown()):
            with self.lockBuffer:
                time_now = rospy.Time.now()
                if len(self.buffer) > 0:
                    self.process_buffer(self.buffer.pop(0))
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
    (options, args) = parser.parse_args()
    
    datalistener = DataListener(options.nodenum)
    datalistener.main()
