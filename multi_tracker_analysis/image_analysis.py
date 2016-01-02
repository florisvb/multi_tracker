#!/usr/bin/env python
from __future__ import division

import numpy as np
import cv2

from optparse import OptionParser

import copy

from scipy import optimize
import data_fit

##############################################################################################
# Circle

def estimate_circle_from_data_points(x_m, y_m):
    model = data_fit.models.CircleModel()
    data = np.zeros(len(x_m))
    model.fit(data, [np.array(x_m), np.array(y_m)])
    model.parameters['radius'] = np.abs(model.parameters['radius'])
    model.parameters['center_x'] = np.abs(model.parameters['center_x'])
    model.parameters['center_y'] = np.abs(model.parameters['center_y'])
    
    print 'Circle Estimates'
    print 'Center (x,y): ', model.parameters['center_x'], model.parameters['center_y']
    print 'Radius: ', model.parameters['radius']
    print
    
    return model.parameters['center_x'], model.parameters['center_y'], model.parameters['radius']
    
    # Iterative Optimization Method
    #print 'Fitting Linear Model with: scipy.optimize.leastsq'
    def f(parameter_values, parameter_names):
        self.set_parameters(parameter_names, parameter_values)
        ans = self.get_errors(data, inputs)
        if len(ans.shape) == 2 and ans.shape[0] == 1:
            ans = ans.reshape(ans.shape[1])
        return ans

    parameter_values = []
    parameter_names = []
    for name, value in self.parameters.items():
        if name in ignore_parameter_names:
            continue
        else:
            parameter_values.append(value)
            parameter_names.append(name)
    optimize.leastsq(f, parameter_values, parameter_names)
    
class ClickCircle(object):
    def __init__(self, filename):
        self.image = cv2.imread(filename)
        
        self.display_name = "Display"
        cv2.namedWindow(self.display_name)
        
        cv2.setMouseCallback(self.display_name, self.on_mouse_click)
        
        self.circle_points_x = []
        self.circle_points_y = []
        self.circle_fit = None
        
    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONUP:
            self.circle_points_x.append(x)
            self.circle_points_y.append(y)
            
        if len(self.circle_points_x) >= 3:
            x,y,R = estimate_circle_from_data_points(self.circle_points_x, self.circle_points_y)
            self.circle_fit = [x,y,R]
            
    def draw(self):
        canvas = copy.copy(self.image)
        for i in range(len(self.circle_points_x)):
            cv2.circle(canvas, (self.circle_points_x[i], self.circle_points_y[i]), 2, [0,0,255], 2)
        if self.circle_fit is not None:
            cv2.circle(canvas, (int(self.circle_fit[0]), int(self.circle_fit[1])), int(self.circle_fit[2]), [0,255,0], 2)
        
        cv2.imshow("Display", canvas)
        #cv2.waitKey(1)
        
    def run(self):
        while (cv2.waitKey(30) != 27):
            self.draw()
        cv.destroyAllWindows();
        
##############################################################################################
# Ellipse

def estimate_ellipse_from_data_points(x_m, y_m):
    points = []
    for i in range(len(x_m)):
        points.append((x_m[i], y_m[i]))
    ellipse = cv2.fitEllipse(np.array(points))
    return ellipse
    
class ClickEllipse(object):
    def __init__(self, filename):
        self.image = cv2.imread(filename)
        
        self.display_name = "Display"
        cv2.namedWindow(self.display_name)
        
        cv2.setMouseCallback(self.display_name, self.on_mouse_click)
        
        self.circle_points_x = []
        self.circle_points_y = []
        self.circle_fit = None
        
    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONUP:
            self.circle_points_x.append(x)
            self.circle_points_y.append(y)
            
        if len(self.circle_points_x) >= 5:
            ellipse = estimate_ellipse_from_data_points(self.circle_points_x, self.circle_points_y)
            self.circle_fit = ellipse
            
    def draw(self):
        canvas = copy.copy(self.image)
        for i in range(len(self.circle_points_x)):
            cv2.circle(canvas, (self.circle_points_x[i], self.circle_points_y[i]), 2, [0,0,255], 2)
        if self.circle_fit is not None:
            print self.circle_fit
            print (int(self.circle_fit[0][0]), int(self.circle_fit[0][1]))
            cv2.ellipse(canvas, (int(self.circle_fit[0][0]), int(self.circle_fit[0][1])), (int(self.circle_fit[1][0]/2.), int(self.circle_fit[1][1]/2.)), int(self.circle_fit[2]), 0, 360, (0,255,0), 2 )
        
        cv2.imshow("Display", canvas)
        #cv2.waitKey(1)
        
    def run(self):
        while (cv2.waitKey(30) != 27):
            self.draw()
        cv.destroyAllWindows();

##############################################################################################
# Pixels

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
        cv2.destroyAllWindows();
            
if __name__ == '__main__':
    
    parser = OptionParser()
    parser.add_option("--filename", type="str", dest="filename", default='',
                        help="filename of image")
    parser.add_option("--analysis", type="str", dest="analysis", default='',
                        help="pixels or circle")
    (options, args) = parser.parse_args()
    
    if options.analysis == 'pixels':
        imageGUI = ClickPixels(options.filename)
    elif options.analysis == 'circle':
        imageGUI = ClickCircle(options.filename)
    elif options.analysis == 'ellipse':
        imageGUI = ClickEllipse(options.filename)
        
    imageGUI.run()
            
