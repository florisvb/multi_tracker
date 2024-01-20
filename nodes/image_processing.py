#!/usr/bin/env python
from __future__ import division
import roslib
import rospy
import rosparam
import copy
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

import time, os

from distutils.version import LooseVersion, StrictVersion
print('Using open cv: ' + cv2.__version__)

# video would not load before installing most recent version of pyqtgraph from github repo
# this is the version of the commit that fixed the
# issue with current numpy: pyqtgraph-0.9.10-118-ge495bbc (in commit e495bbc...)
# version checking with distutils.version. See: http://stackoverflow.com/questions/11887762/compare-version-strings
if StrictVersion(cv2.__version__.split('-')[0]) >= StrictVersion("3.0.0"):
    OPENCV_VERSION = 3
    print('Open CV 3')
else:
    OPENCV_VERSION = 2
    print('Open CV 2')
    
###########################################################################################################
# Incredibly basic image processing function (self contained), to demonstrate the format custom image processing functions should follow
#######################

def incredibly_basic(self):
    # If there is no background image, grab one, and move on to the next frame
    if self.backgroundImage is None:
        reset_background(self)
        return
    if self.reset_background_flag:
        reset_background(self)
        self.reset_background_flag = False
        return
      
    self.absdiff = cv2.absdiff(np.float32(self.imgScaled), self.backgroundImage)
    self.imgproc = copy.copy(self.imgScaled)
    
    retval, self.threshed = cv2.threshold(self.absdiff, self.params['threshold'], 255, 0)
    
    # convert to gray if necessary
    if len(self.threshed.shape) == 3:
        self.threshed = np.uint8(cv2.cvtColor(self.threshed, cv2.COLOR_BGR2GRAY))
    
    # extract and publish contours
    # http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contour_features/py_contour_features.html
    if OPENCV_VERSION == 2:
        contours, hierarchy = cv2.findContours(self.threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    elif OPENCV_VERSION == 3:
        self.threshed = np.uint8(self.threshed)
        contours, hierarchy = cv2.findContours(self.threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    try:
        header  = Header(stamp=self.framestamp,frame_id=str(self.framenumber))
    except:
        header  = Header(stamp=None,frame_id=str(self.framenumber))
        print('could not get framestamp, run tracker_nobuffer instead')
        
    contour_info = []
    for contour in contours:
        if len(contour) > 5: # Large objects are approximated by an ellipse
            ellipse = cv2.fitEllipse(contour)
            (x,y), (a,b), angle = ellipse
            a /= 2.
            b /= 2.
            ecc = np.min((a,b)) / np.max((a,b))
            area = np.pi*a*b
            
            data = Contourinfo()
            data.header  = header
            data.dt      = dtCamera
            data.x       = x
            data.y       = y
            data.area    = area
            data.angle   = angle
            data.ecc     = ecc
            
            contour_info.append(data)
            
        else: # Small ones get ignored
            pass
            
    # publish the contours
    self.pubContours.publish( Contourlist(header = header, contours=contour_info) )  


###########################################################################################################
# General use functions
#######################

def is_point_below_line(point, slope, intercept):
    x = point[0]
    y = point[1]
    result = y-slope*x-intercept
    if result > 0:
        return False
    else:
        return True
    
def fit_ellipse_to_contour(self, contour):
    ellipse = cv2.fitEllipse(contour)
    (x,y), (a,b), angle = ellipse
    a /= 2.
    b /= 2.
    ecc = np.min((a,b)) / np.max((a,b))
    area = np.pi*a*b
    if self.params['use_moments']: # inefficient - double calculating. Is ecc use somewhere else? If not, the ellipse is not at all needed
        try:
            moments = get_centroid_from_moments(contour) # get these values from moments - might be more robust?
        except:
            moments = None
        if moments is not None:
            x, y, area = moments
    return x, y, ecc, area, angle
    
def get_centroid_from_moments(contour):
    M = cv2.moments(contour)
    if M['m00'] != 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        area = cv2.contourArea(contour)
        return cx, cy, area
    else:
        return None
        
def add_data_to_contour_info(x,y,ecc,area,angle,dtCamera,header):
    # Prepare to publish the contour info
    # contour message info: dt, x, y, angle, area, ecc
    data = Contourinfo()
    data.header  = header
    data.dt      = dtCamera
    data.x       = x
    data.y       = y
    data.area    = area
    data.angle   = angle
    data.ecc     = ecc
    return data
    
def extract_and_publish_contours(self):
    if OPENCV_VERSION == 2:
        contours, hierarchy = cv2.findContours(self.threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    elif OPENCV_VERSION == 3:
        contours, hierarchy = cv2.findContours(self.threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # http://docs.opencv.org/trunk/doc/py_tutorials/py_imgproc/py_contours/py_contour_features/py_contour_features.html
    
    try:
        header  = Header(stamp=self.framestamp,frame_id=str(self.framenumber))
    except:
        header  = Header(stamp=None,frame_id=str(self.framenumber))
        print('could not get framestamp, run tracker_nobuffer instead')
        
    contour_info = []
    for contour in contours:
        # Large objects are approximated by an ellipse
        if len(contour) > 5:
            x, y, ecc, area, angle = fit_ellipse_to_contour(self, contour)
            
            # if object is too large, split it in two, this helps with colliding objects, but is not 100% correct
            if area > self.params['max_expected_area']:
                slope = np.tan(angle)
                intercept = y - slope*x
                c1 = []
                c2 = []
                for point in contour:
                    point = point.reshape(2)
                    if is_point_below_line(point, slope, intercept):
                        c1.append([point])
                    else:
                        c2.append([point])
                c1 = np.array(c1)
                c2 = np.array(c2)
                
                if len(c1) > 5:
                    x, y, ecc, area, angle = fit_ellipse_to_contour(self, np.array(c1))
                    if area < self.params['max_size'] and area > self.params['min_size']:
                        data = add_data_to_contour_info(x,y,ecc,area,angle,self.dtCamera,header)
                        contour_info.append(data)
                
                if len(c2) > 5:
                    x, y, ecc, area, angle = fit_ellipse_to_contour(self, np.array(c2))
                    if area < self.params['max_size'] and area > self.params['min_size']:
                        data = add_data_to_contour_info(x,y,ecc,area,angle,self.dtCamera,header)
                        contour_info.append(data)
            else:
                if area < self.params['max_size'] and area > self.params['min_size']:
                    data = add_data_to_contour_info(x,y,ecc,area,angle,self.dtCamera,header)
                    contour_info.append(data)
            
            
        # Small ones just get a point
        else:
            area = 0
            
    # publish the contours
    self.pubContours.publish( Contourlist(header = header, contours=contour_info) )  

    return

def convert_to_gray_if_necessary(self):
    if len(self.threshed.shape) == 3:
        self.threshed = np.uint8(cv2.cvtColor(self.threshed, cv2.COLOR_BGR2GRAY))
        
def erode_and_dialate(self):
    kernel = np.ones((3,3), np.uint8)
    self.threshed = cv2.dilate(self.threshed, kernel, iterations=self.params['dilate'])
    self.threshed = cv2.erode(self.threshed, kernel, iterations=self.params['erode'])
    
def reset_background_if_difference_is_very_large(self, color='dark'):
    if color == 'dark':
        # if the thresholded absolute difference is too large, reset the background
        if np.sum(self.threshed>0) / float(self.shapeImage[0]*self.shapeImage[1]) > self.params['max_change_in_frame']:
            reset_background(self)
            return
    elif color == 'light':
        # NOT IMPLEMENTED
        pass

def reset_background(self):
    self.backgroundImage = copy.copy(np.float32(self.imgScaled))
    print(self.imgScaled.shape, self.backgroundImage.shape)
    filename = self.experiment_basename + '_' + time.strftime("%Y%m%d_%H%M%S_bgimg_N" + self.nodenum, time.localtime()) + '.png'
    home_directory = os.path.expanduser( rospy.get_param('/multi_tracker/' + self.nodenum + '/data_directory') )
    filename = os.path.join(home_directory, filename)
    
    try:
        cv2.imwrite(filename, self.backgroundImage) # requires opencv > 2.4.9
        print('Background reset: ', filename)
    except:
        print('failed to save background image, might need opencv 2.4.9?')

def add_image_to_background(self, color='dark'):
    tmp_backgroundImage = copy.copy(np.float32(self.imgScaled))
    if color == 'dark':
        self.backgroundImage = np.max([self.backgroundImage, tmp_backgroundImage], axis=0)
    elif color == 'light':
        self.backgroundImage = np.min([self.backgroundImage, tmp_backgroundImage], axis=0)
    filename = self.experiment_basename + '_' + time.strftime("%Y%m%d_%H%M%S_bgimg_N" + self.nodenum, time.localtime()) + '.png'
    home_directory = os.path.expanduser( rospy.get_param('/multi_tracker/' + self.nodenum + '/data_directory') )
    filename = os.path.join(home_directory, filename)
    
    try:
        cv2.imwrite(filename, self.backgroundImage) # requires opencv > 2.4.9
        print('Background reset: ', filename)
    except:
        print('failed to save background image, might need opencv 2.4.9?')
###########################################################################################################

###########################################################################################################
# Simple background subtraction
###############################

def background_subtraction(self):
    # If there is no background image, grab one, and move on to the next frame
    if self.backgroundImage is None:
        reset_background(self)
        return
    if self.reset_background_flag:
        reset_background(self)
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
# Only track dark or light objects
#########################

def dark_objects_only(self):
    dark_or_light_objects_only(self, color='dark')

def light_objects_only(self):
    dark_or_light_objects_only(self, color='light')

def dark_or_light_objects(self):
    dark_or_light_objects_only(self, color='darkorlight')

def dark_or_light_objects_only(self, color='dark'):
    if self.params['circular_mask_x'] != 'none':
        if self.image_mask is None:
            self.image_mask = np.zeros_like(self.imgScaled)
            cv2.circle(self.image_mask,(self.params['circular_mask_x'], self.params['circular_mask_y']),int(self.params['circular_mask_r']),[1,1,1],-1)
        self.imgScaled = self.image_mask*self.imgScaled
        
    # If there is no background image, grab one, and move on to the next frame
    if self.backgroundImage is None:
        reset_background(self)
        return
    if self.reset_background_flag:
        reset_background(self)
        self.reset_background_flag = False
        return
    if self.add_image_to_background_flag:
        add_image_to_background(self, color)
        self.add_image_to_background_flag = False
        return 
    
    
    if self.params['backgroundupdate'] != 0:
        cv2.accumulateWeighted(np.float32(self.imgScaled), self.backgroundImage, self.params['backgroundupdate']) # this needs to be here, otherwise there's an accumulation of something in the background
    if self.params['medianbgupdateinterval'] != 0:
        t = rospy.Time.now().secs
        if not self.__dict__.has_key('medianbgimages'):
            self.medianbgimages = [self.imgScaled]
            self.medianbgimages_times = [t]
        if t-self.medianbgimages_times[-1] > self.params['medianbgupdateinterval']:
            self.medianbgimages.append(self.imgScaled)
            self.medianbgimages_times.append(t)
        if len(self.medianbgimages) > 3:
            self.backgroundImage = copy.copy(np.float32(np.median(self.medianbgimages, axis=0)))
            self.medianbgimages.pop(0)
            self.medianbgimages_times.pop(0)
            print('reset background with median image')

    try:
        kernel = self.kernel
    except:
        kern_d = self.params['morph_open_kernel_size']
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kern_d,kern_d))
        self.kernel = kernel
    
    if color == 'dark':
        self.threshed = cv2.compare(np.float32(self.imgScaled), self.backgroundImage-self.params['threshold'], cv2.CMP_LT) # CMP_LT is less than
    elif color == 'light':
        self.threshed = cv2.compare(np.float32(self.imgScaled), self.backgroundImage+self.params['threshold'], cv2.CMP_GT) # CMP_GT is greater than
    elif color == 'darkorlight':
        #absdiff = cv2.absdiff(np.float32(self.imgScaled), self.backgroundImage)
        #retval, self.threshed = cv2.threshold(absdiff, self.params['threshold'], 255, 0)
        #self.threshed = np.uint8(self.threshed)
        dark = cv2.compare(np.float32(self.imgScaled), self.backgroundImage-self.params['threshold'], cv2.CMP_LT) # CMP_LT is less than
        light = cv2.compare(np.float32(self.imgScaled), self.backgroundImage+self.params['threshold'], cv2.CMP_GT) # CMP_GT is greater than
        self.threshed = dark+light
    
    convert_to_gray_if_necessary(self)
    
    # noise removal
    self.threshed = cv2.morphologyEx(self.threshed,cv2.MORPH_OPEN, kernel, iterations = 1)
    
    # sure background area
    #sure_bg = cv2.dilate(opening,kernel,iterations=3)

    # Finding sure foreground area
    #dist_transform = cv2.distanceTransform(opening,cv.CV_DIST_L2,3)
    #dist_transform = dist_transform / (np.max(dist_transform)) * 255
    #ret, sure_fg = cv2.threshold(dist_transform,0.2*dist_transform.max(),255,0)

    # Finding unknown region
    #sure_fg = np.uint8(sure_fg)
    
    #self.threshed = sure_fg
    erode_and_dialate(self)

    # publish the processed image
    c = cv2.cvtColor(np.uint8(self.threshed), cv2.COLOR_GRAY2BGR)
    # commented for now, because publishing unthresholded difference
    
    if OPENCV_VERSION == 2: # cv bridge not compatible with open cv 3, at least at this time
        img = self.cvbridge.cv2_to_imgmsg(c, 'bgr8') # might need to change to bgr for color cameras
        self.pubProcessedImage.publish(img)
    
    extract_and_publish_contours(self)
    #reset_background_if_difference_is_very_large(self, color)
        
