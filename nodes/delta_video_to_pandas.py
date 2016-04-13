#!/usr/bin/env python
from __future__ import division
import roslib
import rospy
import rosparam
import copy
import cv
import cv2
import numpy as np
import threading
import dynamic_reconfigure.server
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Header, String
import imp
import threading

from multi_tracker.msg import Contourinfo, Contourlist, DeltaVid
from multi_tracker.msg import Trackedobject, Trackedobjectlist
from multi_tracker.srv import resetBackgroundService
import os

import pandas
import time
from optparse import OptionParser

import atexit

import multi_tracker_analysis.dvbag_to_pandas_reader as dvbag_to_pandas_reader

# The main tracking class, a ROS node
class DeCompressor:
    def __init__(self, nodenum, saveto='', record_time_hrs=12):
                        
        # initialize the node
        rospy.init_node('delta_to_pandas')
        self.nodename = rospy.get_name().rstrip('/')
        self.time_start = time.time()
        self.saving_data = True
        self.record_time_hrs = record_time_hrs

        # Publishers - publish contours
        topic = '/multi_tracker/' + str(nodenum) + '/delta_video'
        self.subDeltaVid = rospy.Subscriber(topic, DeltaVid, self.save_to_buffer, queue_size=300)
        
        if len(saveto) < 1:
            experiment_basename = rospy.get_param('/multi_tracker/' + nodenum + '/experiment_basename', 'none')
            if experiment_basename == 'none':
                experiment_basename = time.strftime("%Y%m%d_%H%M%S_N" + nodenum, time.localtime())
           
            filename = experiment_basename + '_delta_video_as_pandas.hdf'
            home_directory = os.path.expanduser( rospy.get_param('/multi_tracker/' + nodenum + '/data_directory') )
            filename = os.path.join(home_directory, filename)
            self.saveto = filename
        else:
            self.saveto = saveto

        topic_in = '/multi_tracker/' + str(nodenum) + '/delta_video'
        self.subDeltaVid = rospy.Subscriber(topic_in, DeltaVid, self.save_to_buffer, queue_size=300)
        
        self.buffer = []
        self.lockBuffer = threading.Lock()
        
        self.dvbag = dvbag_to_pandas_reader.DVBag2PandasReader(saveto=saveto)
        
    def save_to_buffer(self, delta_vid):
        self.buffer.append(delta_vid)
        
    def save_delta_image(self, delta_vid):
        if self.saving_data:
            self.dvbag.process_message(delta_vid)
        else:
            pass

        
    def Main(self):
        while (not rospy.is_shutdown()):
            t = time.time() - self.time_start
            if t > self.record_time_hrs*3600:
                self.saving_data = False
                return
            with self.lockBuffer:
                time_now = rospy.Time.now()
                if len(self.buffer) > 0:
                    self.save_delta_image(self.buffer.pop(0))
                pt = (rospy.Time.now()-time_now).to_sec()
                if len(self.buffer) > 20:
                    rospy.logwarn("Data saving processing time exceeds acquisition rate. Processing time: %f, Buffer: %d", pt, len(self.buffer))
            
#####################################################################################################
    
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--saveto", type="str", dest="saveto", default='',
                        help="filename where to save video, default is none")
    parser.add_option("--nodenum", type="int", dest="nodenum", default=1,
                        help="nodenumber")
    parser.add_option("--record-time-hrs", type="int", dest="record_time_hrs", default=12,
                        help="number of hours to record data for")
    (options, args) = parser.parse_args()
    
    decompressor = DeCompressor(options.nodenum, options.saveto)

    decompressor.Main()
