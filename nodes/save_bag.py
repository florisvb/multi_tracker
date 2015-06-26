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


###############################################################################
#
class SaveBag:
    def __init__(self, basename, directory, topics):
        filename = basename + '_' time.strftime('%Y_%m_%d_%H_%M_%S') + '.bag'

        # Make sure path exists.
        try:
            os.makedirs(directory)
        except OSError:
            pass

        self.filenameBag = os.path.join(directory, filename)
        self.topics = topics

        rospy.on_shutdown(self.OnShutdown_callback)

    def OnShutdown_callback(self):
        self.StopRecordingBag()
        
    def StartRecordingBag(self):
        rospy.logwarn('Saving bag file: %s' % (self.dirBag+'/'+self.filenameBag))
        cmdline = ['rosbag', 'record','-O', self.dirBag+'/'+self.filenameBag]
        cmdline.extend([self.topics])
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
    parser.add_option("--directory_parameter_name", type="str", dest="directory_parameter_name", default='',
                        help="name of ros parameter where directory in which to save can be found")
                        
    parser.add_option("--topics_parameter_name", type="str", dest="topics_parameter_name", default='',
                        help="name of ros parameter where list of topics to save can be found")
    
    parser.add_option("--basename", type="str", dest="basename", default='',
                        help="basename for bagfile")
    (options, args) = parser.parse_args()

    directory = rospy.get_param(options.directory_parameter_name)
    topics = rospy.get_param(options.topics_parameter_name)
    
    rospy.init_node('SaveBag', log_level=rospy.INFO)
    rospy.sleep(1)
    savebag = SaveBag(basename, directory, topics)
    savebag.Main()
    
