#!/usr/bin/env python
from __future__ import division
import roslib
import rospy
import os
import time

import numpy as np
from multi_tracker.msg import Trackedobject, Trackedobjectlist

import h5py

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
        
        self.chunk_size = 100
        self.hdf5 = h5py.File(filename, 'w')
        self.hdf5.attrs.create("info", info)
        
        self.data_to_save = [   'header.stamp.secs',
                                'header.stamp.nsecs', 
                                'header.frame_id', 
                                'position', 
                                'velocity',
                                'angle',
                                'size',
                                'covariance',
                                'measurement',
                                ]
        self.data_format = {    'header.stamp.secs': 'int',
                                'header.stamp.nsecs': 'int', 
                                'header.frame_id': 'int', 
                                'position': 'float', 
                                'velocity': 'float',
                                'angle': 'float',
                                'size': 'float',
                                'covariance': 'float',
                                'measurement': 'float',
                            }
        self.data_shape = {    'header.stamp.secs': 1,
                                'header.stamp.nsecs': 1, 
                                'header.frame_id': 1, 
                                'position': 3, 
                                'velocity': 3,
                                'angle': 1,
                                'size': 1,
                                'covariance': 1,
                                'measurement': 2,
                            }
        formats = [self.data_format[data] for data in self.data_to_save]
        
        rospy.init_node('save_data_to_hdf5')
        
    def create_hdf5_object(self, objid, frame_camera):
        self.hdf5.create_group(objid)
        for attribute in self.data_to_save:
            self.hdf5[objid].create_dataset(attribute, (self.chunk_size,self.data_shape[attribute]), maxshape=(None,self.data_shape[attribute]), dtype=self.data_format[attribute])
        self.hdf5[objid].attrs.create('objid', objid)
        self.hdf5[objid].attrs.create('current_frame', 0)
        self.hdf5[objid].attrs.create('first_camera_frame', frame_camera)
        self.hdf5[objid].attrs.create('length', self.chunk_size)
        
    def add_chunk(self, obj):
        length = obj.attrs.get('length')
        new_length = length + self.chunk_size
        for attribute in self.data_to_save:
            obj[attribute].resize(new_length, axis=0)
        obj.attrs.modify('length', new_length)
            
    def save_data(self, obj, tracked_object, frame):
        obj['header.stamp.secs'][frame] = tracked_object.header.stamp.secs
        obj['header.stamp.nsecs'][frame] = tracked_object.header.stamp.nsecs
        obj['header.frame_id'][frame] = int(tracked_object.header.frame_id)
        obj['position'][frame] = [tracked_object.position.x, tracked_object.position.y, tracked_object.position.z]
        obj['velocity'][frame] = [tracked_object.velocity.x, tracked_object.velocity.y, tracked_object.velocity.z]
        obj['angle'][frame] = tracked_object.angle
        obj['size'][frame] = tracked_object.size
        obj['covariance'][frame] = tracked_object.covariance
        obj['measurement'][frame] = [tracked_object.measurement.x, tracked_object.measurement.y]
        
    def tracked_object_callback(self, tracked_objects):
        self.buffer.append(tracked_objects)
        
    def process_buffer(self):
        if len(self.buffer) > 0:
            tracked_objects = self.buffer.pop(0)
            
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
                
                print objid, obj.attrs.get('current_frame'), obj.attrs.get('length'), obj['position'].shape
                 
    def main(self):
        while (not rospy.is_shutdown()):
            self.process_buffer()
        self.hdf5.close()
        
if __name__ == '__main__':
    
    datalistener = DataListener()
    datalistener.main()
