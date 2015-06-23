#!/usr/bin/env python

'''
This node records mouse clicks and keypresses to a csv file. Requires a image topic feed - though that requirement could be relaxed in the future.
'''


from __future__ import division

import numpy as np
import cv
import cv2

from optparse import OptionParser

import copy
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
    
import csv

class ImageGUI(object):
    def __init__(self, filename, image_topic):
        rospy.init_node('record_user_input')
        
        # initialize display
        self.window_name = 'user_input'
        cv2.namedWindow(self.window_name,1)
        self.subImage = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=5, tcp_nodelay=True)
        self.cvbridge = CvBridge()
        
        # initialize file
        self.csvfile = open(filename, 'wb')
        self.datawrite = csv.writer(self.csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)

        # user input
        cv2.setMouseCallback(self.window_name, self.on_mouse_click)
        self.mouse_position = [0,0]

    def image_callback(self, rosimg):
        # Convert the image.
        try:
            img = self.cvbridge.imgmsg_to_cv2(rosimg, 'passthrough') # might need to change to bgr for color cameras
        except CvBridgeError, e:
            rospy.logwarn ('Exception converting background image from ROS to opencv:  %s' % e)
            img = np.zeros((320,240))
        cv2.imshow(self.window_name, img)
        ascii_key = cv2.waitKey(1)
        if ascii_key != -1:
            self.on_key_press(ascii_key)

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()
        self.csvfile.close()
        cv2.destroyAllWindows()
        
    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            data = ['EVENT_LBUTTONDOWN', rospy.get_time(), x, y, 'nokey']
            self.datawrite.writerow(data)
            print data
        if event == cv2.EVENT_MOUSEMOVE:
            self.mouse_position = [x,y]
            
    def on_key_press(self, ascii_key):
        key = chr(ascii_key)
        x,y = self.mouse_position
        data = ['KEYPRESS', rospy.get_time(), x, y, key]
        self.datawrite.writerow(data)
        print data
    
if __name__ == '__main__':
    
    parser = OptionParser()
    parser.add_option("--image", type="str", dest="image", default='',
                        help="topic of image")
                        
    parser.add_option("--filename", type="str", dest="filename", default='',
                        help="name of file to which user input will be recorded")
                        
    (options, args) = parser.parse_args()
            
    imageGUI = ImageGUI(options.filename, options.image)
    imageGUI.run()
            
            
            
            
            
    
