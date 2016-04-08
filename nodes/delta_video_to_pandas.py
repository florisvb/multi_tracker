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
    def __init__(self, nodenum, saveto=''):
                        
        # initialize the node
        rospy.init_node('delta_to_pandas')
        self.nodename = rospy.get_name().rstrip('/')
        
        # Publishers - publish contours
        topic = '/multi_tracker/' + str(nodenum) + '/delta_video'
        self.subDeltaVid = rospy.Subscriber(topic, DeltaVid, self.delta_image_callback, queue_size=300)
        
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
        
        self.dvbag = dvbag_to_pandas_reader.DVBag2PandasReader(saveto=saveto)
        
    def delta_image_callback(self, delta_vid):
        self.dvbag.process_message(delta_vid)
        
    def Main(self):
        rospy.spin()
            
#####################################################################################################
    
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--saveto", type="str", dest="saveto", default='',
                        help="filename where to save video, default is none")
    parser.add_option("--nodenum", type="int", dest="nodenum", default=1,
                        help="nodenumber")
    
    (options, args) = parser.parse_args()
    
    decompressor = DeCompressor(options.nodenum, options.saveto)

    decompressor.Main()
