#!/usr/bin/env python
from __future__ import division
from optparse import OptionParser
import roslib
import rospy
import rosparam
import copy
import numpy as np
import os, sys
import time

from std_msgs.msg import Float32, Header, String                                                                                                                                  
from geometry_msgs.msg import Point, Vector3
from multi_tracker.msg import Contourinfo, Contourlist
from multi_tracker.msg import Trackedobject, Trackedobjectlist

import matplotlib.pyplot as plt
import multi_tracker_analysis.Kalman as Kalman
import imp

import threading


class DataAssociator(object):
    def __init__(self, nodenum):
        kalman_parameter_py_file = rospy.get_param('/multi_tracker/' + nodenum + '/data_association/kalman_parameters_py_file')
        home_directory = os.path.expanduser( rospy.get_param('/multi_tracker/' + nodenum + '/home_directory') )
        kalman_parameter_py_file = os.path.join(home_directory, kalman_parameter_py_file)
        print 'Kalman py file: ', kalman_parameter_py_file
        
        self.lockBuffer = threading.Lock()
        
        self.kalman_parameters = imp.load_source('kalman_parameters', kalman_parameter_py_file)
        self.association_matrix = self.kalman_parameters.association_matrix
        self.association_matrix /= np.linalg.norm(self.association_matrix)
        self.max_covariance = self.kalman_parameters.max_covariance
        self.max_velocity = self.kalman_parameters.max_velocity
        
        self.tracked_objects = {}
        self.current_objid = 0
        
        #self.min_size = rospy.get_param('/multi_tracker/data_association/min_size')
        #self.max_size = rospy.get_param('/multi_tracker/data_association/max_size')
        self.max_tracked_objects = rospy.get_param('/multi_tracker/' + nodenum + '/data_association/max_tracked_objects')
        self.n_covariances_to_reject_data = rospy.get_param('/multi_tracker/' + nodenum + '/data_association/n_covariances_to_reject_data')

        self.contour_buffer = []
        
        # initialize the node
        rospy.init_node('data_associator_' + nodenum)
        self.time_start = time.time()
        
        # Publishers.
        self.pubTrackedObjects = rospy.Publisher('/multi_tracker/' + nodenum + '/tracked_objects', Trackedobjectlist, queue_size=300)
        
        # Subscriptions.
        self.subImage = rospy.Subscriber('/multi_tracker/' + nodenum + '/contours', Contourlist, self.contour_callback, queue_size=300)
        
    def contour_callback(self, contourlist):
        with self.lockBuffer:
            self.contour_buffer.append(contourlist)
        
    def contour_identifier(self, contourlist):
        
        # keep track of which new objects have been "taken"
        contours_accounted_for = []
        
        update_dict = {}
        
        def update_tracked_object(tracked_object, measurement, contourlist):
            if measurement is None:
                m = np.matrix([np.nan for i in range( tracked_object['measurement'].shape[0] ) ]).T
                xhat, P, K = tracked_object['kalmanfilter'].update( None ) # run kalman filter
            else:
                tracked_object['measurement'] = np.hstack( (tracked_object['measurement'], measurement) ) # add object's data to the tracked object
                xhat, P, K = tracked_object['kalmanfilter'].update( tracked_object['measurement'][:,-1] ) # run kalman filter
            tracked_object['frames'].append(int(contourlist.header.frame_id))
            tracked_object['frames'].pop(0)
            tracked_object['nframes'] += 1
            tracked_object['timestamp'].append(contourlist.header.stamp)
            tracked_object['timestamp'].pop(0)
            tracked_object['state'] = np.hstack( (tracked_object['state'][:,-1], xhat) )
            
        
        # iterate through objects first
        # get order of persistence
        objid_in_order_of_persistance = []
        if len(self.tracked_objects.keys()) > 0:
            persistance = []
            objids = []
            for objid, tracked_object in self.tracked_objects.items():
                 persistance.append(tracked_object['nframes'])
                 objids.append(objid)
            order = np.argsort(persistance)[::-1]
            objid_in_order_of_persistance = [objids[o] for o in order]
        
                    
        # loop through contours and find errors to all tracked objects (if less than allowed error)
        # then loop through the errors in order of increasing error and assigned contours to objects
        contour_to_object_error = []
        tracked_object_state_estimates = None
        tracked_object_covariances = None 
        tracked_object_ids = []            
        for objid, tracked_object in self.tracked_objects.items():
            tose = np.array([[tracked_object['kalmanfilter'].xhat_apriori[0,0], tracked_object['kalmanfilter'].xhat_apriori[2,0]]]) 
            cov = np.array([np.linalg.norm( (tracked_object['kalmanfilter'].H*tracked_object['kalmanfilter'].P).T*self.association_matrix )])
            tracked_object_ids.append(objid)
            if tracked_object_state_estimates is None:
                tracked_object_state_estimates = tose
                tracked_object_covariances = cov
            else:
                tracked_object_state_estimates = np.vstack((tracked_object_state_estimates,  tose))
                tracked_object_covariances = np.vstack((tracked_object_covariances,  cov))
            
        if tracked_object_state_estimates is not None:
            for c, contour in enumerate(contourlist.contours):
                m = np.array([[contour.x, contour.y]])
                error = np.array([np.linalg.norm(m-e) for e in tracked_object_state_estimates])
                ncov = self.n_covariances_to_reject_data*np.sqrt(tracked_object_covariances)
                indices = np.where( (error < ncov ) )[0]
                if len(indices) > 0:
                    new_contour_object_errors = [[error[i], tracked_object_ids[i], c] for i in indices]
                    contour_to_object_error.extend(new_contour_object_errors)
                    
        # Association and Propagation                
        #o = []
        if len(contour_to_object_error) > 0:
            contour_to_object_error = np.array(contour_to_object_error)
            sorted_indices = np.argsort(contour_to_object_error[:,0])
            contour_to_object_error = contour_to_object_error[sorted_indices,:]
            contours_accounted_for = []
            objects_accounted_for = []
            for data in contour_to_object_error:
                c = int(data[2])
                objid = int(data[1])
                if objid not in objects_accounted_for:
                    if c not in contours_accounted_for:
                        contour = contourlist.contours[c]
                        measurement = np.matrix([contour.x, contour.y, 0, contour.area, contour.angle]).T
                        tracked_object = self.tracked_objects[objid]
                        update_tracked_object(tracked_object, measurement, contourlist)
                        contours_accounted_for.append(c)
                        objects_accounted_for.append(objid)
                        #e = [   tracked_object['kalmanfilter'].xhat_apriori[0] - contour.x,
                        #        tracked_object['kalmanfilter'].xhat_apriori[2] - contour.y]
                        #tracked_object_covariance = np.linalg.norm( (tracked_object['kalmanfilter'].H*tracked_object['kalmanfilter'].P).T*self.association_matrix )
                        #o.append([objid, e, tracked_object_covariance])
                                     
        # any unnaccounted contours should spawn new objects
        for c, contour in enumerate(contourlist.contours):
            if c not in contours_accounted_for:
                
                obj_state = np.matrix([contour.x, 0, contour.y, 0, 0, 0, contour.area, 0, contour.angle, 0]).T # pretending 3-d tracking (z and zvel) for now
                obj_measurement = np.matrix([contour.x, contour.y, 0, contour.area, contour.angle]).T
                # If not associated with previous object, spawn a new object
                new_obj = { 'objid':        self.current_objid,
                            'statenames':   {   'position': [0, 2, 4], 
                                                'velocity': [1, 3, 5],
                                                'size': 6,
                                                'd_size': 7,
                                                'angle': 8,
                                                'd_angle': 9,    
                                            },
                            'state':        obj_state,
                            'measurement':  obj_measurement,
                            'timestamp':    [contour.header.stamp],
                            'frames':       [int(contour.header.frame_id)],
                            'kalmanfilter': Kalman.DiscreteKalmanFilter(x0      = obj_state, 
                                                                        P0      = self.kalman_parameters.P0, 
                                                                        phi     = self.kalman_parameters.phi, 
                                                                        gamma   = self.kalman_parameters.gamma, 
                                                                        H       = self.kalman_parameters.H, 
                                                                        Q       = self.kalman_parameters.Q, 
                                                                        R       = self.kalman_parameters.R, 
                                                                        gammaW  = self.kalman_parameters.gammaW,
                                                                        ),
                            'nframes':      0,
                          }
                self.tracked_objects.setdefault(new_obj['objid'], new_obj)
                self.current_objid += 1
        
        
        
        # propagate unmatched objects
        for objid, tracked_object in self.tracked_objects.items():
            if tracked_object['frames'][-1] != int(contourlist.header.frame_id):
                update_tracked_object(tracked_object, None, contourlist)
        
        # make sure we don't get too many objects - delete the oldest ones, and ones with high covariances
        objects_to_destroy = []
        if len(objid_in_order_of_persistance) > self.max_tracked_objects:
            for objid in objid_in_order_of_persistance[self.max_tracked_objects:]:
                objects_to_destroy.append(objid)
        
        # check covariance, and velocity
        for objid, tracked_object in self.tracked_objects.items():
            tracked_object_covariance = np.linalg.norm( (tracked_object['kalmanfilter'].H*tracked_object['kalmanfilter'].P).T*self.association_matrix )
            if tracked_object_covariance > self.max_covariance:
                if objid not in objects_to_destroy:
                    objects_to_destroy.append(objid)
            v = np.linalg.norm( np.array( tracked_object['state'][tracked_object['statenames']['velocity'],-1] ).flatten().tolist() )
            if v > self.max_velocity:
                if objid not in objects_to_destroy:
                    objects_to_destroy.append(objid)
        for objid in objects_to_destroy:
            del(self.tracked_objects[objid])
            
        # recalculate persistance (not necessary, but convenient)
        objid_in_order_of_persistance = []
        if len(self.tracked_objects.keys()) > 0:
            persistance = []
            for objid, tracked_object in self.tracked_objects.items():
                 persistance.append(len(tracked_object['frames']))
                 objid_in_order_of_persistance.append(objid)
            order = np.argsort(persistance)[::-1]
            objid_in_order_of_persistance = [objid_in_order_of_persistance[o] for o in order]
        if len(objid_in_order_of_persistance) > 0:
            most_persistant_objid = objid_in_order_of_persistance[0]

        # publish tracked objects
        if 1:
            object_info_to_publish = []
            t = contourlist.header.stamp
            for objid in objid_in_order_of_persistance:
                if objid not in objects_to_destroy:
                    tracked_object = self.tracked_objects[objid]
                    data = Trackedobject()
                    data.header  = Header(stamp=t, frame_id=contourlist.header.frame_id)
                    p = np.array( tracked_object['state'][tracked_object['statenames']['position'],-1] ).flatten().tolist()
                    v = np.array( tracked_object['state'][tracked_object['statenames']['velocity'],-1] ).flatten().tolist()
                    data.position       = Point( p[0], p[1], p[2] )
                    data.velocity       = Vector3( v[0], v[1], v[2] )
                    data.angle          = tracked_object['state'][tracked_object['statenames']['angle'],-1]
                    data.size           = tracked_object['state'][tracked_object['statenames']['size'],-1]#np.linalg.norm(tracked_object['kalmanfilter'].P.diagonal())
                    data.measurement    = Point( tracked_object['measurement'][0, -1], tracked_object['measurement'][1, -1], 0)
                    tracked_object_covariance = np.linalg.norm( (tracked_object['kalmanfilter'].H*tracked_object['kalmanfilter'].P).T*self.association_matrix )
                    data.covariance     = tracked_object_covariance # position covariance only
                    data.objid          = tracked_object['objid']
                    data.persistence    = tracked_object['nframes']
                    object_info_to_publish.append(data)
            header = Header(stamp=t)
            self.pubTrackedObjects.publish( Trackedobjectlist(header=header, tracked_objects=object_info_to_publish) )
        
    def main(self):
        while not rospy.is_shutdown():
            t = time.time() - self.time_start
            if t > 24*3600:
                return
            with self.lockBuffer:
                time_now = rospy.Time.now()
                if len(self.contour_buffer) > 0:
                    self.contour_identifier(self.contour_buffer.pop(0))
                pt = (rospy.Time.now()-time_now).to_sec()
                if len(self.contour_buffer) > 9:
                    rospy.logwarn("Data association processing time exceeds acquisition rate. Processing time: %f, Buffer: %d", pt, len(self.contour_buffer))
            
            
                
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--nodenum", type="str", dest="nodenum", default='1',
                        help="node number, for example, if running multiple tracker instances on one computer")
    (options, args) = parser.parse_args()
    
    data_associator = DataAssociator(options.nodenum)
    data_associator.main()
