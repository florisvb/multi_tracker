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

###########################################################################################################
# General use functions
#######################

def extract_and_publish_contours(self):
    contours, hierarchy = cv2.findContours(self.threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # http://docs.opencv.org/trunk/doc/py_tutorials/py_imgproc/py_contours/py_contour_features/py_contour_features.html
    
    contour_info = []
    for contour in contours:
        # Large objects are approximated by an ellipse
        if len(contour) > 5:
            ellipse = cv2.fitEllipse(contour)
            cv2.ellipse(self.imgOutput,ellipse,(0,255,0),2) # draw the ellipse, green
            (x,y), (a,b), angle = ellipse
            a /= 2.
            b /= 2.
            ecc = np.min((a,b)) / np.max((a,b))
            area = np.pi*a*b
        # Small ones just get a point
        else:
            moments = cv2.moments(contour, True)
            m00 = moments['m00']
            m10 = moments['m10']
            m01 = moments['m01']
            if (m00 != 0.0):
                x = m10/m00
                y = m01/m00
            else: # There was just one pixel in the contour.
                (x,y) = contour[0][0]
            area = 1.
            angle = 0.
            ecc = 1.
            cv2.circle(self.imgOutput,(int(x),int(y)),2,(0,255,0),2) # draw a circle, green
            
        # Prepare to publish the contour info
        # contour message info: dt, x, y, angle, area, ecc
        data = Contourinfo()
        data.header  = Header(seq=self.iCountCamera,stamp=rospy.Time.now(),frame_id='BackgroundSubtraction')
        data.dt      = self.dtCamera
        data.x       = x
        data.y       = y
        data.area    = area
        data.angle   = angle
        data.ecc     = ecc
        contour_info.append(data)
            
    # publish the contours
    self.pubContours.publish( Contourlist(header = Header(seq=self.iCountCamera,stamp=rospy.Time.now(),frame_id='BackgroundSubtraction'), contours=contour_info) )  

    return

def convert_to_gray_if_necessary(self):
    if self.params['camera_encoding'] == 'mono8':
        self.threshed = np.uint8(self.threshed)
    else:
        self.threshed = np.uint8(cv2.cvtColor(self.threshed, cv2.COLOR_BGR2GRAY))
        
def erode_and_dialate(self):
    kernel = np.ones((5,5),np.uint8)
    self.threshed = cv2.erode(self.threshed, kernel, iterations=self.params['erode'])
    self.threshed = cv2.dilate(self.threshed, kernel, iterations=self.params['dilate'])
    
def reset_background_if_difference_is_very_large(self):
    # if the thresholded absolute difference is too large, reset the background
    if np.sum(self.threshed>0) / float(self.shapeImage[0]*self.shapeImage[1]) > self.params['max_change_in_frame']:
        self.backgroundImage = copy.copy(np.float32(self.imgScaled))
        return
###########################################################################################################

###########################################################################################################
# Simple background subtraction
###############################

def background_subtraction(self):
    now = rospy.get_time()
    
    # If there is no background image, grab one, and move on to the next frame
    if self.backgroundImage is None:
        self.backgroundImage = copy.copy(np.float32(self.imgScaled))
        return
    if self.reset_background_flag:
        self.backgroundImage = copy.copy(np.float32(self.imgScaled))
        self.reset_background_flag = False
        return
      
    # Absdiff, threshold, and contours       
    # cv2.RETR_EXTERNAL only extracts the outer most contours - good for speed, and most simple objects 
    self.absdiff = cv2.absdiff(np.float32(self.imgScaled), self.backgroundImage)
    self.imgproc = copy.copy(self.imgScaled)
    # running background update
    cv2.accumulateWeighted(np.float32(self.imgScaled), self.backgroundImage, self.params['backgroundupdate']) # this needs to be here, otherwise there's an accumulation of something in the background
    
    retval, self.threshed = cv2.threshold(self.absdiff, self.params['threshold'], 255, 0)
    
    convert_to_gray_if_necessary(self)
    erode_and_dialate(self)
    extract_and_publish_contours(self)
    reset_background_if_difference_is_very_large(self)
        
###########################################################################################################
# Only track dark objects
#########################

def dark_objects_only(self):
    now = rospy.get_time()
    
    # If there is no background image, grab one, and move on to the next frame
    if self.backgroundImage is None:
        self.backgroundImage = copy.copy(np.float32(self.imgScaled))
        return
    if self.reset_background_flag:
        self.backgroundImage = copy.copy(np.float32(self.imgScaled))
        self.reset_background_flag = False
        return
      
    self.threshed = cv2.compare(np.float32(self.imgScaled), self.backgroundImage-self.params['threshold'], cv2.CMP_LT) # CMP_LT is less than
        
    convert_to_gray_if_necessary(self)
    erode_and_dialate(self)
    extract_and_publish_contours(self)
    reset_background_if_difference_is_very_large(self)
        
###########################################################################################################
# Track objects based on mean absolute difference, and standard deviations. 
# Good for highly sensitive tracking in a noisy environment. Loses objects that stop moving. 
##########################

def calculate_standard_deviations_for_each_pixel(self):
    n = np.min([self.n_frames_processed, 1/self.stdDevUpdate])
    sum_x = self.meanDifference*n
    sum_x_sq = n*(self.stdDifference**2 + self.meanDifference**2)
    stdDifference_sq = ( (n+1)*(self.difference**2 + sum_x_sq) - (self.difference + sum_x)**2 ) / (n+1)**2
    self.stdDifference = np.float32(np.sqrt(stdDifference_sq))
    cv2.accumulateWeighted(self.difference, self.meanDifference, 1/float(n))
    
def background_subtraction_with_standard_deviations(self):
    
    # If there is no background image, grab one, and move on to the next frame
    if self.backgroundImage is None:
        self.backgroundImage = copy.copy(np.float32(self.imgScaled))
        self.meanDifference = copy.copy(self.backgroundImage)
        self.stdDifference = np.zeros_like(self.meanDifference)
        self.n_frames_processed = 1
        
        self.stdDevUpdate = rospy.get_param('/multi_tracker/tracker/std_dev_update')
        return
    if self.reset_background_flag:
        self.backgroundImage = copy.copy(np.float32(self.imgScaled))
        self.meanDifference = copy.copy(self.backgroundImage)
        self.stdDifference = np.zeros_like(self.meanDifference)
        self.n_frames_processed = 1
        
        self.stdDevUpdate = rospy.get_param('/multi_tracker/tracker/std_dev_update')
        
        self.reset_background_flag = False
        return
    
    self.n_frames_processed += 1
    
    # calculate difference between img and background, and add to pixel by pixel standard deviation measurement
    self.difference = cv2.add(np.float32(self.imgScaled), -1*self.backgroundImage)
    calculate_standard_deviations_for_each_pixel(self)
    
    # running background update
    self.imgproc = copy.copy(np.uint8(self.stdDifference))
    cv2.accumulateWeighted(np.float32(self.imgScaled), self.backgroundImage, self.params['backgroundupdate']) # this needs to be here, otherwise there's an accumulation of something in the background

    # if difference - meandifference is larger than N standard deviations, it is a pixel of interest
    absdiff = np.float32(cv2.absdiff(self.difference, self.meanDifference))
    n_devs_away = cv2.divide(absdiff, self.stdDifference)
    self.threshed = cv2.compare(n_devs_away, self.params['threshold'], cv2.CMP_GT) # CMP_GT is great than
        
    print self.n_frames_processed, np.min(n_devs_away), np.max(n_devs_away)
        
    convert_to_gray_if_necessary(self)
    erode_and_dialate(self)
    extract_and_publish_contours(self)

####################################################################################
