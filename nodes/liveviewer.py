#!/usr/bin/env python
from __future__ import division
from optparse import OptionParser
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
from multi_tracker.srv import resetBackgroundService, addImageToBackgroundService

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
class LiveViewer:
    def __init__(self, nodenum):
        '''
        Default image_topic for:
            Basler ace cameras with camera_aravis driver: camera/image_raw
            Pt Grey Firefly cameras with pt grey driver : camera/image_mono
        '''
        # default parameters (parameter server overides them)
        self.params = { 'image_topic'               : '/camera/image_mono',
                        'min_persistence_to_draw'   : 10,
                        'max_frames_to_draw'        : 50,
                        'camera_encoding'           : 'mono8', # fireflies are bgr8, basler gige cams are mono8
                        'roi_l'                     : 0,
                        'roi_r'                     : -1,
                        'roi_b'                     : 0,
                        'roi_t'                     : -1,
                        'circular_mask_x'           : 'none',
                        'circular_mask_y'           : 'none',
                        'circular_mask_r'           : 'none',
                        }
        for parameter, value in self.params.items():
            try:
		# allows image processed view to be overlaid with tracked objects
                p = '/multi_tracker/' + nodenum + '/liveviewer/' + parameter
                self.params[parameter] = rospy.get_param(p)
            except:
                try:
                    p = '/multi_tracker/' + nodenum + '/tracker/' + parameter
                    self.params[parameter] = rospy.get_param(p)
                except:
                    print 'Using default parameter: ', parameter, ' = ', value
                
        # initialize the node
        rospy.init_node('liveviewer_' + nodenum)
        self.nodename = rospy.get_name().rstrip('/')
        self.nodenum = nodenum
        
        # initialize display
        self.window_name = 'output'
        cv2.namedWindow(self.window_name,1)
        self.subTrackedObjects = rospy.Subscriber('/multi_tracker/' + nodenum + '/tracked_objects', Trackedobjectlist, self.tracked_object_callback)
        self.subContours = rospy.Subscriber('/multi_tracker/' + nodenum + '/contours', Contourlist, self.contour_callback)
            
        self.cvbridge = CvBridge()
        self.tracked_trajectories = {}
        self.contours = None
        cv2.setMouseCallback(self.window_name, self.on_mouse_click)
        
        # Subscriptions - subscribe to images, and tracked objects
        self.image_mask = None 
        sizeImage = 128+1024*1024*3 # Size of header + data.
        self.subImage = rospy.Subscriber(self.params['image_topic'], Image, self.image_callback, queue_size=5, buff_size=2*sizeImage, tcp_nodelay=True)

        # for adding images to background
        add_image_to_background_service_name = '/multi_tracker/' + self.nodenum + '/' + 'tracker/' + "add_image_to_background"
        rospy.wait_for_service(add_image_to_background_service_name)
        try:
            self.add_image_to_background = rospy.ServiceProxy(add_image_to_background_service_name, addImageToBackgroundService)
        except:
            print 'could not connect to add image to background service - is tracker running?'

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

    def contour_callback(self, contours):
        self.contours = contours

    def image_callback(self, rosimg):
        # Convert the image.
        try:
            img = self.cvbridge.imgmsg_to_cv2(rosimg, 'passthrough') # might need to change to bgr for color cameras
        except CvBridgeError, e:
            rospy.logwarn ('Exception converting background image from ROS to opencv:  %s' % e)
            img = np.zeros((320,240))

        self.imgScaled = img[self.params['roi_b']:self.params['roi_t'], self.params['roi_l']:self.params['roi_r']]
        self.shapeImage = self.imgScaled.shape # (height,width)
        
        if self.params['circular_mask_x'] != 'none':
            if self.image_mask is None:
                self.image_mask = np.zeros_like(self.imgScaled)
                cv2.circle(self.image_mask,(self.params['circular_mask_x'], self.params['circular_mask_y']),int(self.params['circular_mask_r']),[1,1,1],-1)
            self.imgScaled = self.image_mask*self.imgScaled
        
        # Image for display
        if self.params['camera_encoding'] == 'mono8':
            self.imgOutput = cv2.cvtColor(self.imgScaled, cv2.COLOR_GRAY2RGB)
        elif self.params['camera_encoding'] == 'binary':
            self.imgOutput = self.imgScaled
        else:
            self.imgOutput = self.imgScaled
        
        # Draw ellipses from contours
        if self.contours is not None:
            for c, contour in enumerate(self.contours.contours):
                # b = contour.area / (np.pi*a)
                # b = ecc*a
                a = np.sqrt( contour.area / (np.pi*contour.ecc) )
                b = contour.ecc*a
                center = (int(contour.x), int(contour.y))
                angle = int(contour.angle)
                axes = (int(np.min([a,b])), int(np.max([a,b])))
                cv2.ellipse(self.imgOutput, center, axes, angle, 0, 360, (0,255,0), 2 )
                            
        # Display the image | Draw the tracked trajectories
        for objid, trajec in self.tracked_trajectories.items():
            if len(trajec.positions) > 5:
                draw_trajectory(self.imgOutput, trajec.positions, trajec.color, 2)
                cv2.circle(self.imgOutput,(int(trajec.positions[-1][0]),int(trajec.positions[-1][1])),int(trajec.covariances[-1]),trajec.color,2)
        cv2.imshow('output', self.imgOutput)
        ascii_key = cv2.waitKey(1)
        if ascii_key != -1:
            self.on_key_press(ascii_key)
        
    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONUP:
            print 'clicked pixel: ', [x, y]
    
    def on_key_press(self, ascii_key):
        key = chr(ascii_key)
        if key == 'a':
            resp = self.add_image_to_background()
            print 'added image to background'
            
    def Main(self):
        while (not rospy.is_shutdown()):
            rospy.spin()
        cv2.destroyAllWindows()

#####################################################################################################
    
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--nodenum", type="str", dest="nodenum", default='1',
                        help="node number, for example, if running multiple tracker instances on one computer")
    (options, args) = parser.parse_args()
    
    liveviewer = LiveViewer(options.nodenum)
    liveviewer.Main()
