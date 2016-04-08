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
    def __init__(self, topic_in, saveto=''):
                        
        # initialize the node
        rospy.init_node('delta_to_pandas')
        self.nodename = rospy.get_name().rstrip('/')
        self.time_start = time.time()
        self.saving_data = True
        self.record_time = 12
        # Publishers - publish contours
        self.subDeltaVid = rospy.Subscriber(topic_in, DeltaVid, self.save_to_buffer, queue_size=300)
        
        self.buffer = []
        self.lockBuffer = threading.Lock()
        
        self.saveto = saveto
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
                    self.process_buffer(self.image_buffer.pop(0))
                pt = (rospy.Time.now()-time_now).to_sec()
                if len(self.buffer) > 9:
                    rospy.logwarn("Data saving processing time exceeds acquisition rate. Processing time: %f, Buffer: %d", pt, len(self.buffer))
            
#####################################################################################################
    
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--in", type="str", dest="input", default='/multi_tracker/delta_video',
                        help="input topic name")
    parser.add_option("--saveto", type="str", dest="saveto", default='',
                        help="filename where to save video, default is none")
    
    (options, args) = parser.parse_args()
    
    decompressor = DeCompressor(options.input, options.saveto)

    decompressor.Main()
