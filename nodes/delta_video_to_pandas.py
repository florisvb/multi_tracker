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

from multi_tracker.msg import Contourinfo, Contourlist, DeltaVid
from multi_tracker.msg import Trackedobject, Trackedobjectlist
from multi_tracker.srv import resetBackgroundService
import os

import pandas

from optparse import OptionParser

import atexit

import multi_tracker_analysis.dvbag_to_pandas_reader as dvbag_to_pandas_reader

# The main tracking class, a ROS node
class DeCompressor:
    def __init__(self, topic_in, saveto=''):
                        
        # initialize the node
        rospy.init_node('delta_to_pandas')
        self.nodename = rospy.get_name().rstrip('/')
        
        # Publishers - publish contours
        self.subDeltaVid = rospy.Subscriber(topic_in, DeltaVid, self.delta_image_callback, queue_size=300)
        
        self.saveto = saveto
        self.dvbag = dvbag_to_pandas_reader.DVBag2PandasReader(saveto=saveto)
        
    def delta_image_callback(self, delta_vid):
        self.dvbag.process_message(delta_vid)
        
    def Main(self):
        rospy.spin()
            
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
