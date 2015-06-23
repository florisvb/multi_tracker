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

from multi_tracker.msg import Contourinfo, Contourlist
from multi_tracker.msg import Trackedobject, Trackedobjectlist
from multi_tracker.srv import resetBackgroundService

import image_processing

# for basler ace cameras, use camera_aravis
# https://github.com/ssafarik/camera_aravis
# rosrun camera_aravis camnode
# default image: /camera/image_raw

# for firefley cameras, camera1394 does not provide timestamps but otherwise works
# use point grey drivers
# http://wiki.ros.org/pointgrey_camera_driver
# rosrun pointgrey_camera_driver camera_node
# default image: /camera/image_mono

# Trajectory class to aid in drawing colored tracked trajectories with opencv
class Trajectory(object):
    def __init__(self, objid):
        self.objid = objid
        self.positions = []
        self.color = None
        self.covariances = []
        self.popout = 0

def draw_trajectory(img, pts, color, thickness):
    for i in range(len(pts)-3):
        try:
            cv2.line(img, (int(pts[i][0]), int(pts[i][1])), (int(pts[i+1][0]), int(pts[i+1][1])), color, thickness)
        except:
            pass
            print 'could not draw trajectory line, length pts: ', len(pts), 'i: ', i
            
# The main tracking class, a ROS node
class Tracker:
    def __init__(self):
        '''
        Default image_topic for:
            Basler ace cameras with camera_aravis driver: camera/image_raw
            Pt Grey Firefly cameras with pt grey driver : camera/image_mono
        '''
        # default parameters (parameter server overides them)
        self.params = { 'image_topic'       : '/camera/image_mono',
                        'threshold'         : 20,
                        'backgroundupdate'  : 0.001,
                        'camera_encoding'   : 'mono8', # fireflies are bgr8, basler gige cams are mono8
                        'min_persistence_to_draw'   : 10,
                        'max_frames_to_draw'        : 50,
                        'erode'                     : 1,
                        'dilate'                    : 2,
                        'max_change_in_frame'       : 0.2,
                        'min_size'                  : 5,
                        'max_size'                  : 200,
                        }
        for parameter, value in self.params.items():
            try:
                p = '/multi_tracker/tracker/' + parameter
                self.params[parameter] = rospy.get_param(p)
            except:
                print 'Using default parameter: ', parameter, ' = ', value
                        
        # set up thread locks
        self.lockBuffer = threading.Lock()
        
        # initialize the node
        rospy.init_node('multi_tracker')
        self.nodename = rospy.get_name().rstrip('/')
        
        # background reset service
        self.reset_background_flag = False
        self.reset_background_service = rospy.Service("/multi_tracker/reset_background", resetBackgroundService, self.reset_background)
        
        # initialize display
        self.window_name = 'output'
        cv2.namedWindow(self.window_name,1)
        self.window_name = 'imgproc'
        #cv2.namedWindow(self.window_name,2)
        #self.imgproc = None
        
        self.cvbridge = CvBridge()
        
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
        
        self.bufferImages   = [None]*30 # Circular buffer for incoming images.
        self.iImgLoading    = 0  # Index of the next slot to load.
        self.iImgWorking    = 0  # Index of the slot to process, i.e. the oldest image in the buffer.
        self.imgUnscaled    = None
        self.imgScaled      = None
        
        self.backgroundImage = None
        self.tracked_trajectories = {}
        
        # Publishers - publish contours
        self.pubContours = rospy.Publisher('/multi_tracker/contours', Contourlist, queue_size=30)
        
        # Subscriptions - subscribe to images, and tracked objects
        sizeImage = 128+1024*1024*3 # Size of header + data.
        self.subImage = rospy.Subscriber(self.params['image_topic'], Image, self.image_callback, queue_size=5, buff_size=2*sizeImage, tcp_nodelay=True)
        #self.subTrackedObjects = rospy.Subscriber('/multi_tracker/tracked_objects', Trackedobjectlist, self.tracked_object_callback)
        self.subTrackedObjects = rospy.Subscriber('/multi_tracker/tracked_objects', Trackedobjectlist, self.tracked_object_callback)

    def reset_background(self, service_call):
        self.reset_background_flag = True
        return 1
        
    def tracked_object_callback(self, tracked_objects):
        for trajec in self.tracked_trajectories.values():
            trajec.popout = 1
            
        for tracked_object in tracked_objects.tracked_objects:
            if tracked_object.persistence > self.params['min_persistence_to_draw']:
                if tracked_object.objid not in self.tracked_trajectories.keys(): # create new object
                    self.tracked_trajectories.setdefault(tracked_object.objid, Trajectory(tracked_object.objid))
                    self.tracked_trajectories[tracked_object.objid].color = np.random.randint(0,255,3).tolist()
                # update tracked objects
                self.tracked_trajectories[tracked_object.objid].covariances.append(tracked_object.covariance)
                self.tracked_trajectories[tracked_object.objid].positions.append([tracked_object.position.x, tracked_object.position.y])
                
                # if it is a young object, let it grow to length 100
                if len(self.tracked_trajectories[tracked_object.objid].positions) < self.params['max_frames_to_draw']:
                    self.tracked_trajectories[tracked_object.objid].popout = 0
        
        # cull old objects
        for objid, trajec in self.tracked_trajectories.items():
            if trajec.popout:
                trajec.positions.pop(0)
                trajec.covariances.pop(0)
                if len(trajec.positions) <= 1:
                    del(self.tracked_trajectories[objid])
                                    
        
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
                print 'DROPPED FRAME: ', self.iDroppedFrame
                
            # Put the image into the queue.
            self.bufferImages[self.iImgLoading] = rosimg
            self.iImgLoading = iImgLoadingNext
            self.iImgWorking = iImgWorkingNext
        self.bValidImage = True

    def process_frame_buffer(self):
        rosimg = None
        
        time_now = rospy.Time.now()
        
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
            
            # Image for display
            if self.params['camera_encoding'] == 'mono8':
                self.imgOutput = cv2.cvtColor(self.imgScaled, cv2.COLOR_GRAY2RGB)
            else:
                self.imgOutput = self.imgScaled
            
########### Call to image processing function ##############################################################
            self.process_image() # must be defined seperately - see "main" code at the bottom of this script
############################################################################################################
            
            # Display the image.
            # Draw the tracked trajectories
            #print self.tracked_trajectories.keys()
            
            if 0:
                for objid, trajec in self.tracked_trajectories.items():
                    if len(trajec.positions) > 5:
                        draw_trajectory(self.imgOutput, trajec.positions, trajec.color, 2)
                        #print objid, trajec.color
                        cv2.circle(self.imgOutput,(int(trajec.positions[-1][0]),int(trajec.positions[-1][1])),int(trajec.covariances[-1]),trajec.color,2)
                        #print trajec.covariances[-1]
                        
                # Show the image
                #cv2.imshow('output', self.imgOutput)
                cv2.imshow('output', self.imgOutput)
                
            #if self.imgproc is not None:
            #    cv2.imshow('imgproc', self.imgproc)
            
            #print self.stampCamera.secs, rospy.Time.now().secs
            
        cv2.waitKey(1)
        
        pt = (rospy.Time.now()-time_now).to_sec()
        if len(self.bufferImages) > 3:
            rospy.logwarn("Tracker processing time exceeds acquisition rate. Processing time: %f, Buffer: %d", pt, len(self.bufferImages))
                    
                    
    def Main(self):
        while (not rospy.is_shutdown()):
            self.process_frame_buffer()
        cv2.destroyAllWindows()

#####################################################################################################
    
if __name__ == '__main__':
    
    image_processing_function = rospy.get_param('/multi_tracker/tracker/image_processor')
    image_processor = image_processing.__getattribute__(image_processing_function)
    Tracker.process_image = image_processor
    tracker = Tracker()
    tracker.Main()
