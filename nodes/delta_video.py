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
class Compressor:
    def __init__(self):
        '''
        Default image_topic for:
            Basler ace cameras with camera_aravis driver: camera/image_raw
            Pt Grey Firefly cameras with pt grey driver : camera/image_mono
        '''
        # default parameters (parameter server overides them)
        self.params = { 'image_topic'       : '/camera/image_raw',
                        'threshold'         : 10,
                        'camera_encoding'   : 'mono8', # fireflies are bgr8, basler gige cams are mono8
                        'max_change_in_frame'       : 0.2,
                        }
                        
        # initialize the node
        rospy.init_node('delta_compressor')
        self.nodename = rospy.get_name().rstrip('/')
        
        # Publishers - publish contours
        self.pubDeltaVid = rospy.Publisher('/multi_tracker/delta_video', DeltaVid, queue_size=30)
        
        # background reset service
        self.reset_background_flag = False
        self.reset_background_service = rospy.Service("/multi_tracker/reset_background", resetBackgroundService, self.reset_background)
        
        self.cvbridge = CvBridge()
        
        
        # set up thread locks
        self.lockParams = threading.Lock()
        self.lockBuffer = threading.Lock()
        # more parameters
        self.scale          = 1
        self.stampCameraPrev= rospy.Time(0)
        self.stampCameraDiff= rospy.Duration(0)
        self.stampROSPrev   = rospy.Time(0)
        self.stampROSDiff   = rospy.Duration(0)
        self.stampMax       = rospy.Duration(0)
        self.dtCamera       = np.inf
        self.hzCameraF      = 0.0
        self.hzCameraSum    = 0.0
        self.hzROSF         = 0.0
        self.hzROSSum       = 0.0
        self.iCountCamera   = 0
        self.iCountROS      = 0
        self.iDroppedFrame  = 0
        
        self.nQueuePrev     = 0     # Length of the image queue.
        self.dnQueueF       = 0.0   # Rate of change of the image queue length.
        
        self.bufferImages   = [None]*2 # Circular buffer for incoming images.
        self.iImgLoading    = 0  # Index of the next slot to load.
        self.iImgWorking    = 0  # Index of the slot to process, i.e. the oldest image in the buffer.
        self.imgUnscaled    = None
        self.imgScaled      = None
        
        self.backgroundImage = None
        self.background_img_filename = 'none'
        
        self.current_background_img = 0
        
        # Subscriptions - subscribe to images, and tracked objects
        sizeImage = 128+1024*1024*3 # Size of header + data.
        self.subImage = rospy.Subscriber(self.params['image_topic'], Image, self.image_callback, queue_size=5, buff_size=2*sizeImage, tcp_nodelay=True)

    def reset_background(self, service_call):
        self.reset_background_flag = True
        return 1
        
    def image_callback(self, rosimg):
        '''
        Save image to buffer for processing, when the processor gets a chance
        '''
        # Receive the image:
        with self.lockBuffer:
            # Check for dropped frame.
            if (self.bufferImages[self.iImgLoading] is None):   # There's an empty slot in the buffer.
                iImgLoadingNext = (self.iImgLoading+1) % len(self.bufferImages)
                iImgWorkingNext = self.iImgWorking
                self.iDroppedFrame = 0
            else:                                               # The buffer is full; we'll overwrite the oldest entry.
                iImgLoadingNext = (self.iImgLoading+1) % len(self.bufferImages)
                iImgWorkingNext = (self.iImgWorking+1) % len(self.bufferImages)
                self.iDroppedFrame += 1

            # Put the image into the queue.
            self.bufferImages[self.iImgLoading] = rosimg
            self.iImgLoading = iImgLoadingNext
            self.iImgWorking = iImgWorkingNext
        self.bValidImage = True

    def process_frame_buffer(self):
        rosimg = None
        
        with self.lockBuffer:
            # The image queue length.
            nQueue = (self.iImgLoading - self.iImgWorking) %  len(self.bufferImages)
            if (nQueue==0) and (self.bufferImages[self.iImgLoading] is not None):
                nQueue += len(self.bufferImages)
                        
            # Rate of change of the queue length.
            if (nQueue == len(self.bufferImages)):
                dnQueue = 1.0
            elif (nQueue <= 1):
                dnQueue = -1.0
            else:
                dnQueue = nQueue - self.nQueuePrev
    
            # Bring the bar back to red, if it's green and we dropped a frame.
            if (self.iDroppedFrame>0) and (self.dnQueueF<0.0):
                self.dnQueueF = 0.1
            else:  
                a = 0.001
                self.dnQueueF = (1-a)*self.dnQueueF + a*dnQueue
            self.aQueue = float(nQueue)/float(len(self.bufferImages))
            self.nQueuePrev = nQueue
                    
            # Pull the image from the queue.
            if (self.bufferImages[self.iImgWorking] is not None):
                rosimg = self.bufferImages[self.iImgWorking]
                
                # Mark this buffer entry as available for loading.
                self.bufferImages[self.iImgWorking] = None
    
                # Go to the next image.
                self.iImgWorking = (self.iImgWorking+1) % len(self.bufferImages)
                
        # Compute processing times.
        self.stampROS        = rospy.Time.now()
        self.stampROSDiff    = (self.stampROS - self.stampROSPrev)
        self.stampROSPrev    = self.stampROS
        self.dtROS           = max(0, self.stampROSDiff.to_sec())

        # If time wrapped, then just assume a value.
        if (self.dtROS == 0.0):
            self.dtROS = 1.0

        # Compute system freq.
        hzROS = 1/self.dtROS
        self.iCountROS += 1
        if (self.iCountROS > 100):                     
            a= 0.04 # Filter the framerate.
            self.hzROSF = (1-a)*self.hzROSF + a*hzROS 
        else:                                       
            if (self.iCountROS>20):             # Get past the transient response.       
                self.hzROSSum += hzROS                 
            else:
                self.hzROSSum = hzROS * self.iCountROS     
            self.hzROSF = self.hzROSSum / self.iCountROS
        
        if (rosimg is not None):            
            # Compute processing times.
            self.stampCamera     = rosimg.header.stamp
            self.stampCameraDiff = (self.stampCamera - self.stampCameraPrev)
            self.stampCameraPrev = self.stampCamera
            self.dtCamera        = max(0, self.stampCameraDiff.to_sec())

            # If the camera is not giving good timestamps, then use our own clock.
            if (self.dtCamera == 0.0):
                self.dtCamera = self.dtROS
                
            # If time wrapped, then just assume a value.
            if (self.dtCamera == 0.0):
                self.dtCamera = 1.0
                    
            # Compute processing freq.
            hzCamera = 1/self.dtCamera
            self.iCountCamera += 1
            if (self.iCountCamera > 100):                     
                a= 0.01 # Filter the framerate.
                self.hzCameraF = (1-a)*self.hzCameraF + a*hzCamera 
            else:                                       
                if (self.iCountCamera>20):             # Get past the transient response.       
                    self.hzCameraSum += hzCamera                 
                else:
                    self.hzCameraSum = hzCamera * self.iCountCamera     
                self.hzCameraF = self.hzCameraSum / self.iCountCamera
                        
            # Convert the image.
            try:
                img = self.cvbridge.imgmsg_to_cv2(rosimg, 'passthrough') # might need to change to bgr for color cameras
            except CvBridgeError, e:
                rospy.logwarn ('Exception converting background image from ROS to opencv:  %s' % e)
                img = np.zeros((320,240))

            # Scale the image - not implemented
            if 0:
                if self.scale != 1:
                    raise ValueError('resizing not implemented')
                self.imgUnscaled = img
                if (self.scale == 1.0):              
                    self.imgScaled = self.imgUnscaled
                else:  
                    self.imgScaled = cv2.resize(img, (0,0), fx=self.scale, fy=self.scale) 
            self.imgScaled = img
            self.shapeImage = self.imgScaled.shape # (height,width)
            
########### image processing function ##############################################################
        
            # If there is no background image, grab one, and move on to the next frame
            if self.backgroundImage is None:
                self.backgroundImage = copy.copy(self.imgScaled)
                self.background_img_filename = str(self.current_background_img) + '.png'
                cv2.imwrite(self.background_img_filename, self.backgroundImage)
                self.current_background_img += 1
                return
            if self.reset_background_flag:
                self.backgroundImage = copy.copy(self.imgScaled)
                self.background_img_filename = str(self.current_background_img) + '.png'
                cv2.imwrite(self.background_img_filename, self.backgroundImage)
                self.current_background_img += 1
                self.reset_background_flag = False
                return
            # Absdiff
            self.absdiff = cv2.absdiff(self.imgScaled, self.backgroundImage)
            
            changed_pixels = np.where(self.absdiff>self.params['threshold'])
            delta_msg = DeltaVid()
            delta_msg.header.stamp = self.stampCamera 
            delta_msg.background_image = self.background_img_filename
            delta_msg.xpixels = changed_pixels[0].tolist()
            delta_msg.ypixels = changed_pixels[1].tolist()
            delta_msg.values = self.imgScaled[changed_pixels].tolist()
            self.pubDeltaVid.publish(delta_msg)
            
            # if the thresholded absolute difference is too large, reset the background
            if len(changed_pixels[0]) / (self.absdiff.shape[0]*self.absdiff.shape[1])>self.params['max_change_in_frame']:
                self.reset_background_flag = True
    
            #self.backgroundImage[delta_msg.xpixels, delta_msg.ypixels] = delta_msg.values 
    
    def Main(self):
        while (not rospy.is_shutdown()):
            self.process_frame_buffer()
        cv2.destroyAllWindows()

#####################################################################################################
    
if __name__ == '__main__':
    
    compressor = Compressor()
    compressor.Main()
