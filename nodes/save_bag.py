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
# import cv: open cv 1 not used
from cv_bridge import CvBridge, CvBridgeError

import imp

###############################################################################
#
class SaveBag:
    def __init__(self, config, nodenum):
        basename = config.basename
        directory = config.directory
        self.topics = config.topics
        try:
            self.record_length_seconds = config.record_length_hours*3600
        except:
            self.record_length_seconds = 24*3600
        self.time_start = time.time()
        
        try:
            filename = config.filename  
        except:
            try:
                experiment_basename = rospy.get_param('/multi_tracker/' + nodenum + '/experiment_basename', 'none')
                if experiment_basename == 'none':
                    experiment_basename = time.strftime("%Y%m%d_%H%M%S_N" + nodenum, time.localtime())
                
                filename = experiment_basename + '_' + basename + '.bag'
            except:
                raise ValueError('Need to define filename in config, or use the multi_tracker infrastructure')

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
        rate = rospy.Rate(0.01)
        while not rospy.is_shutdown():
            t = time.time() - self.time_start
            if t > self.record_length_seconds:
                self.StopRecordingBag()      
                return
        

if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--config", type="str", dest="config", default='',
                        help="filename of configuration file")
    parser.add_option("--nodenum", type="str", dest="nodenum", default='1',
                        help="node number, for example, if running multiple tracker instances on one computer")
    (options, args) = parser.parse_args()
    
    
    try:
        configuration = imp.load_source('configuration', options.config)
        print "Loaded configuration: ", options.config
    except: # look in home directory for config file
        home_directory = os.path.expanduser( rospy.get_param('/multi_tracker/' + options.nodenum + '/home_directory') )
        config_file = os.path.join(home_directory, options.config)
        configuration = imp.load_source('configuration', config_file)
        print "Loaded configuration: ", config_file
    
    config = configuration.Config()

    rospy.init_node('SaveBag', log_level=rospy.INFO)
    rospy.sleep(1)
    savebag = SaveBag(config, nodenum=options.nodenum)
    savebag.Main()
    
