import multi_tracker_analysis.read_hdf5_file_to_pandas as mta_read

import cv2
import numpy as np
import pickle
import os

import copy
from optparse import OptionParser

from distutils.version import LooseVersion, StrictVersion
print 'Using open cv: ' + cv2.__version__
if StrictVersion(cv2.__version__.split('-')[0]) >= StrictVersion("3.0.0"):
    OPENCV_VERSION = 3
    print 'Open CV 3'
else:
    OPENCV_VERSION = 2
    print 'Open CV 2'

def create_median_gray_small_image_from_directory(directory, N=3, resize_factor=0.1):
    # N is the number of equidistant files to use for making the median
    file_list = mta_read.get_filenames(directory, '.jpg')
    imgs = []
    indices = np.linspace(0,len(file_list)-1,N).astype(int)
    for i in indices:
        file = file_list[i]
        img = cv2.imread(file, cv2.CV_8UC1)
        small = cv2.resize(img, (0,0), fx=resize_factor, fy=resize_factor) 
        imgs.append(small)
    median = np.median(imgs, axis=0)
    return median.astype(np.uint8)

def find_fly_in_image(image, median, threshold=10, pixels_per_mm=10, min_fly_length_mm=1, max_fly_ecc=5):
    '''
    pixels_per_mm - after resize transformation
    '''
    absdiff = cv2.absdiff(image, median)
    retval, threshed = cv2.threshold(absdiff, threshold, 255, 0)
    
    kern_d = 3
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kern_d,kern_d))
    threshed = cv2.morphologyEx(threshed,cv2.MORPH_OPEN, kernel, iterations = 1)
    
    kernel = np.ones((3,3), np.uint8)
    threshed = cv2.dilate(threshed, kernel, iterations=1)
    threshed = cv2.erode(threshed, kernel, iterations=2)

    # http://docs.opencv.org/trunk/doc/py_tutorials/py_imgproc/py_contours/py_contour_features/py_contour_features.html
    if OPENCV_VERSION == 2:
        contours, hierarchy = cv2.findContours(threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    elif OPENCV_VERSION == 3:
        threshed = np.uint8(threshed)
        image, contours, hierarchy = cv2.findContours(threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    fly_ellipses = []
    for contour in contours:
        if len(contour) > 5:
            ellipse = cv2.fitEllipse(contour)
            fly_length = np.max(ellipse[1])
            fly_width = np.min(ellipse[1])
            fly_ecc = fly_length / fly_width
            if fly_length > min_fly_length_mm and fly_ecc < max_fly_ecc:
                fly_ellipses.append(ellipse)
    
    return fly_ellipses


def find_flies_in_images(directory, 
                         resize_factor=0.2, 
                         threshold=10, 
                         pixels_per_mm=10, 
                         min_fly_length_mm=1, 
                         max_fly_ecc=5,
                         save_result=True):
    
    median = create_median_gray_small_image_from_directory(directory,  
                                                           N=3, 
                                                           resize_factor=resize_factor)
    
    flies = {}
    file_list = mta_read.get_filenames(directory, '.jpg')
    for file in file_list:
        print file
        img = cv2.imread(file, cv2.CV_8UC1)
        small = cv2.resize(img, (0,0), fx=resize_factor, fy=resize_factor) 
    
        print small.shape
        print median.shape
        fly_ellipses = find_fly_in_image(small, 
                                         median, 
                                         threshold=threshold, 
                                         pixels_per_mm=pixels_per_mm, 
                                         min_fly_length_mm=min_fly_length_mm, 
                                         max_fly_ecc=max_fly_ecc)
        
        large_fly_ellipses = []
        for ellipse in fly_ellipses:
            large_ellipse = ((ellipse[0][0]/resize_factor, ellipse[0][1]/resize_factor), 
                             (ellipse[1][0]/resize_factor, ellipse[1][1]/resize_factor), 
                             ellipse[2])
            large_fly_ellipses.append(large_ellipse)
        
        flies[file] = large_fly_ellipses
        
    if save_result:
        fname = os.path.join(directory, 'fly_ellipses.pickle')
        f = open(fname, 'w')
        pickle.dump(flies, f)
        f.close()
        

if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option('--path', type=str, default='none', help="option: path that points to directory of images")
    (options, args) = parser.parse_args()

    find_flies_in_images(options.path)