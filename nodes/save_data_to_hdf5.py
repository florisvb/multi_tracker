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
        
        self.chunk_size = 100
        self.hdf5 = open(filename, 'w')
        self.hdf5.attrs.create("info", info)
        
        self.data_to_save = [   'header.stamp.secs',
                                'header.stamp.nsecs', 
                                'header.seq', 
                                'position', 
                                'velocity',
                                'angle',
                                'size',
                                'covariance',
                                'measurement',
                                ]
        self.data_format = {    'header.stamp.secs': 'int',
                                'header.stamp.nsecs': 'int', 
                                'header.seq': 'int', 
                                'position': 'float', 
                                'velocity': 'float',
                                'angle': 'float',
                                'size': 'float',
                                'covariance': 'float',
                                'measurement': 'float',
                            }
        self.data_shape = {    'header.stamp.secs': 1,
                                'header.stamp.nsecs': 1, 
                                'header.seq': 1, 
                                'position': 2, 
                                'velocity': 2,
                                'angle': 1,
                                'size': 1,
                                'covariance': 1,
                                'measurement': 2,
                            }
        formats = [self.data_format[data] for data in self.data_to_save]
        
        rospy.init_node('save_data_to_hdf5')
        
    def create_hdf5_object(self, objid):
        self.hdf5.create_group(objid)
        for attribute in self.data_to_save:
            self.hdf5[objid].create_dataset(attribute, (self.chunk_size,self.data_shape[attribute]), maxshape=(None,self.data_shape[attribute]), dtype=self.data_format[attribute])
        self.hdf5[objid].attrs.create('objid', objid)
        self.hdf5[objid].attrs.create('current_frame', 0)
        self.hdf5[objid].attrs.create('length', self.chunk_size)
        
    def add_chunk(self, obj):
        for attribute in self.data_to_save:
            obj[attribute].resize(obj.attrs.get('length')+self.chunk_size, axis=0)
            obj.attrs.modify('length', obj.attrs.get('length')+self.chunk_size)
            
    def save_data(self, obj, tracked_object, frame):
        obj['header.stamp.secs'][frame] = tracked_object.header.stamp.secs
        obj['header.stamp.nsecs'][frame] = tracked_object.header.stamp.nsecs
        obj['header.seq'][frame] = tracked_object.header.seq
        obj['position'][frame] = [tracked_object.position.x, tracked_object.position.y, tracked_object.position.z]
        obj['velocity'][frame] = [tracked_object.velocity.x, tracked_object.velocity.y, tracked_object.velocity.z]
        obj['angle'][frame] = tracked_object.angle
        obj['size'][frame] = tracked_object.size
        obj['covariance'][frame] = tracked_object.covariance
        obj['measurement'][frame] = [tracked_object.measurement.x, tracked_object.measurement.y]
        
        
    def tracked_object_callback(self, tracked_objects):
        for tracked_object in tracked_objects.tracked_objects:
            objid = str(tracked_object.objid)
            if objid not in self.hdf5:
                self.hdf5.create_group(objid)
            
            obj = self.hdf5[objid]
            obj.attrs.modify('current_frame', obj.attrs.get('current_frame') + 1)
            frame = obj.attrs.get('current_frame')
            
            if frame > obj.attrs.get('length'):
                self.add_chunk(obj)
                
            self.save_data(obj, tracked_object, frame)
                    
    def main(self):
        while (not rospy.is_shutdown()):
            rospy.spin()
        self.csvfile.close()
        
if __name__ == '__main__':
    
    datalistener = DataListener()
    datalistener.main()
