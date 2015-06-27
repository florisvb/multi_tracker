#!/usr/bin/env python
from __future__ import division
import rospy
from optparse import OptionParser
import tf
import sys
import time, os, subprocess
import threading
import numpy as np

from sensor_msgs.msg import Image
import cv
from cv_bridge import CvBridge, CvBridgeError

import imp

###############################################################################
#
class SaveBag:
    def __init__(self, config):
        basename = config.basename
        directory = config.directory
        self.topics = config.topics
        filename = basename + '_' + time.strftime('%Y_%m_%d_%H_%M_%S') + '.bag'

        # Make sure path exists.
        try:
            os.makedirs(directory)
        except OSError:
            pass

        self.filenameBag = os.path.expanduser(os.path.join(directory, filename))
        self.processRosbag = None
        
        rospy.on_shutdown(self.OnShutdown_callback)

    def OnShutdown_callback(self):
        self.StopRecordingBag()
        
    def StartRecordingBag(self):
        rospy.logwarn('Saving bag file: %s' % (self.filenameBag))
        cmdline = ['rosbag', 'record','-O', self.filenameBag]
        cmdline.extend(self.topics)
        print cmdline
        self.processRosbag = subprocess.Popen(cmdline, preexec_fn=subprocess.os.setpgrp)
    
    def StopRecordingBag(self):
        subprocess.os.killpg(self.processRosbag.pid, subprocess.signal.SIGINT)
        rospy.logwarn('Closed bag file.')
                
    def Main(self):
        savebag.StartRecordingBag()
        while (not rospy.is_shutdown()):
            rospy.spin()            
        

if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--config", type="str", dest="config", default='',
                        help="filename of configuration file")
    (options, args) = parser.parse_args()
    
    configuration = imp.load_source('configuration', options.config)
    config = configuration.Config()

    rospy.init_node('SaveBag', log_level=rospy.INFO)
    rospy.sleep(1)
    savebag = SaveBag(config)
    savebag.Main()
    
