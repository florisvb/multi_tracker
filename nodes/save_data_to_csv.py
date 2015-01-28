#!/usr/bin/env python
from __future__ import division
import roslib
import rospy
import os

import numpy as np
from multi_tracker.msg import Trackedobject, Trackedobjectlist

import csv

class DataListener:
    def __init__(self, info='data information'):
        self.subTrackedObjects = rospy.Subscriber('multi_tracker/tracked_objects', Trackedobjectlist, self.tracked_object_callback)
        
        filename = rospy.get_param('/multi_tracker/csv_data_filename')
        home_directory = os.path.expanduser( rospy.get_param('/multi_tracker/data_directory') )
        filename = os.path.join(home_directory, filename)
        
        self.csvfile = open(filename, 'wb')
        self.datawrite = csv.writer(self.csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        
        self.data_to_save = [   'header.stamp.secs',
                                'header.stamp.nsecs', 
                                'objid', 
                                'header.seq', 
                                'position.x', 
                                'position.y',
                                'position.z',
                                'velocity.x',
                                'velocity.y',
                                'velocity.z',
                                'angle',
                                'size',
                                'covariance',
                                ]
        self.data_format = {    'header.stamp.secs': 'int',
                                'header.stamp.nsecs': 'int', 
                                'objid': 'str', 
                                'header.seq': 'int', 
                                'position.x': 'float', 
                                'position.y': 'float',
                                'position.z': 'float',
                                'velocity.x': 'float',
                                'velocity.y': 'float',
                                'velocity.z': 'float',
                                'angle': 'float',
                                'size': 'float',
                                'covariance': 'float',
                            }
        formats = [self.data_format[data] for data in self.data_to_save]
        
        self.datawrite.writerow([info])
        self.datawrite.writerow(self.data_to_save)
        self.datawrite.writerow(formats)
        rospy.init_node('save_data_to_csv')

    def tracked_object_callback(self, tracked_objects):
        for tracked_object in tracked_objects.tracked_objects:
            row = []
            for attribute in self.data_to_save:
                attributes = attribute.split('.')
                a = tracked_object.__getattribute__(attributes[0])
                for attribute in attributes[1:]:
                    a = a.__getattribute__(attribute)
                row.append( a )
                    
                    
            '''
            tracked_object.header.stamp, 
                    tracked_object.objid, 
                    tracked_object.header.seq, 
                    tracked_object.position.x, 
                    tracked_object.position.y, 
                    tracked_object.position.z,
                    tracked_object.velocity.x,
                    tracked_object.velocity.y,
                    tracked_object.velocity.z,
                    tracked_object.angle,
                    tracked_object.size,
                    tracked_object.covariance,
                    ]  
            ''' 
            self.datawrite.writerow(row)
            print row
            
    def main(self):
        while (not rospy.is_shutdown()):
            rospy.spin()
        self.csvfile.close()
        
if __name__ == '__main__':
    
    datalistener = DataListener()
    datalistener.main()
