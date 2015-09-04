#!/usr/bin/env python
from __future__ import division

import numpy as np
import cv
import cv2

from optparse import OptionParser

import copy


class ClickPixels(object):
    def __init__(self, filename):
        self.image = cv2.imread(filename)
        
        self.display_name = "Display"
        cv2.namedWindow(self.display_name)
        
        cv2.setMouseCallback(self.display_name, self.on_mouse_click)
        
        self.points = []
        
    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONUP:
            print x, y
            self.points.append([x,y])
            
    def draw(self):
        canvas = copy.copy(self.image)
        for point in self.points:
            cv2.circle(canvas, (point[0], point[1]), 2, [0,0,255], 2)
        cv2.imshow("Display", canvas)
        #cv2.waitKey(1)
        
    def run(self):
        while (cv2.waitKey(30) != 27):
            self.draw()
        cv.destroyAllWindows();
            
if __name__ == '__main__':
    
    parser = OptionParser()
    parser.add_option("--filename", type="str", dest="filename", default='',
                        help="filename of image")
    parser.add_option("--analysis", type="str", dest="analysis", default='',
                        help="pixels or circles")
    (options, args) = parser.parse_args()
    
    if options.analysis == 'pixels':
        imageGUI = ClickPixels(options.filename)
    
    
    imageGUI.run()
            
