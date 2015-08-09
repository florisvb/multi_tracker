#!/usr/bin/env python
from __future__ import division
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
    def __init__(self, info='data information'):
        self.subTrackedObjects = rospy.Subscriber('multi_tracker/tracked_objects', Trackedobjectlist, self.tracked_object_callback)
        
        filename = rospy.get_param('/multi_tracker/csv_data_filename')
        if filename == 'none':
            filename = time.strftime("%Y%m%d_%H%M_rotpaddata", time.localtime()) + '.hdf5'
        home_directory = os.path.expanduser( rospy.get_param('/multi_tracker/data_directory') )
        filename = os.path.join(home_directory, filename)
        print 'Saving hdf5 data to: ', filename
        
        self.buffer = []
        # set up thread locks
        self.lockParams = threading.Lock()
        self.lockBuffer = threading.Lock()
        
        self.chunk_size = 500
        self.hdf5 = h5py.File(filename, 'w')
        self.hdf5.attrs.create("info", info)
        
        self.data_to_save = [   'header.stamp.secs',
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
        self.data_format = {    'header.stamp.secs': int,
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
        rospy.init_node('save_data_to_hdf5')
        
    def create_hdf5_object(self, objid, frame_camera):
        self.hdf5.create_dataset(objid, (self.chunk_size, 1), maxshape=(None,1), dtype=self.dtype)
        self.hdf5[objid].attrs.create('objid', objid)
        self.hdf5[objid].attrs.create('current_frame', 0)
        self.hdf5[objid].attrs.create('first_camera_frame', frame_camera)
        self.hdf5[objid].attrs.create('length', self.chunk_size)
        
    def add_chunk(self, obj):
        length = obj.attrs.get('length')
        new_length = length + self.chunk_size
        obj.resize(new_length, axis=0)
        obj.attrs.modify('length', new_length)
            
    def save_data(self, obj, tracked_object, frame):
        obj[frame] = np.array([(    tracked_object.header.stamp.secs,
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
            
            frame_camera = int(tracked_object.header.frame_id)
            
            objid = str(tracked_object.objid)
            if objid not in self.hdf5:
                self.create_hdf5_object(objid, frame_camera)
            
            obj = self.hdf5[objid]
            frame_obj = frame_camera - obj.attrs.get('first_camera_frame')
            
            if frame_obj >= obj.attrs.get('length'):
                self.add_chunk(obj)
            
            self.save_data(obj, tracked_object, frame_obj)
            obj.attrs.modify('current_frame',  frame_obj+1)
            
            #print objid, obj.attrs.get('current_frame'), obj.attrs.get('length'), obj['position'].shape
            
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
    
    datalistener = DataListener()
    datalistener.main()
