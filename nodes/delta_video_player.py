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

from multi_tracker.msg import Contourinfo, Contourlist, DeltaVid
from multi_tracker.msg import Trackedobject, Trackedobjectlist
from multi_tracker.srv import resetBackgroundService
import os

from optparse import OptionParser

import image_processing

import matplotlib.pyplot as plt

# for basler ace cameras, use camera_aravis
# https://github.com/ssafarik/camera_aravis
# rosrun camera_aravis camnode
# default image: /camera/image_raw

# for firefley cameras, camera1394 does not provide timestamps but otherwise works
# use point grey drivers
# http://wiki.ros.org/pointgrey_camera_driver
# rosrun pointgrey_camera_driver camera_node
# default image: /camera/image_mono

# The main tracking class, a ROS node
class DeCompressor:
    def __init__(self, topic_in, topic_out, directory):
        '''
        Default image_topic for:
            Basler ace cameras with camera_aravis driver: camera/image_raw
            Pt Grey Firefly cameras with pt grey driver : camera/image_mono
        '''
        # default parameters (parameter server overides them)
                        
        # initialize the node
        rospy.init_node('delta_decompressor')
        self.nodename = rospy.get_name().rstrip('/')
        
        # Publishers - publish contours
        self.pubDeltaVid = rospy.Publisher(topic_out, Image, queue_size=30)
        self.subDeltaVid = rospy.Subscriber(topic_in, DeltaVid, self.delta_image_callback, queue_size=30)
        
        self.directory = directory #rospy.get_param('/multi_tracker/delta_video/directory', default='')
        
        self.cvbridge = CvBridge()
        
        self.backgroundImage = None
        self.background_img_filename = 'none'
        
    def delta_image_callback(self, delta_vid):
        if self.background_img_filename != delta_vid.background_image:
            self.background_img_filename = delta_vid.background_image
            basename = os.path.basename(self.background_img_filename)
            directory_with_basename = os.path.join(self.directory, basename)
            self.backgroundImage = cv2.imread(directory_with_basename, cv2.CV_8UC1)
            try:
                self.backgroundImage = self.backgroundImage.reshape([self.backgroundImage.shape[0], self.backgroundImage[1], 1]) # for hydro
            except:
                pass # for indigo
                
        new_image = copy.copy(self.backgroundImage)
        
        if delta_vid.values is not None:
            if len(delta_vid.values) > 0:
                try:
                    new_image[delta_vid.xpixels, delta_vid.ypixels, 0] = delta_vid.values # for hydro
                except:
                    new_image[delta_vid.xpixels, delta_vid.ypixels] = delta_vid.values # for indigo

        image_message = self.cvbridge.cv2_to_imgmsg(new_image, encoding="mono8")
        image_message.header = delta_vid.header
        self.pubDeltaVid.publish(image_message)
    
    def Main(self):
        rospy.spin()

#####################################################################################################
    
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--in", type="str", dest="input", default='/multi_tracker/delta_video',
                        help="input topic name")
    parser.add_option("--out", type="str", dest="output", default='/camera/image_decompressed',
                        help="output topic name")
    parser.add_option("--directory", type="str", dest="directory", default='',
                        help="directory where background images can be found")
    (options, args) = parser.parse_args()
    
    decompressor = DeCompressor(options.input, options.output, options.directory)
    decompressor.Main()
