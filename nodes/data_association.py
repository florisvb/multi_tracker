#!/usr/bin/env python
from __future__ import division
import roslib
import rospy
import rosparam
import copy
import numpy as np
from std_msgs.msg import Float32, Header, String
from geometry_msgs.msg import Point, Vector3
from multi_tracker.msg import Contourinfo, Contourlist
from multi_tracker.msg import Trackedobject, Trackedobjectlist

import matplotlib.pyplot as plt
import Kalman

class Struct(object):
    pass

class DataAssociator(object):
    def __init__(self, kalman_parameters):
        self.kalman_parameters = kalman_parameters
        self.tracked_objects = {}
        self.current_objid = 0
        
        self.association_matrix = np.matrix([[1,1,0,0,0]], dtype=float).T
        self.association_matrix /= np.linalg.norm(self.association_matrix)
        self.min_size = 5
        self.max_covariance = np.linalg.norm(self.kalman_parameters.P0)*10
        self.max_tracked_objects = 5
        
        # initialize the node
        rospy.init_node('data_associator')
        
        # Publishers.
        self.pubTrackedObjects = rospy.Publisher('multi_tracker/tracked_objects', Trackedobjectlist)
        
        # Subscriptions.
        self.subImage = rospy.Subscriber('multi_tracker/contours', Contourlist, self.contour_identifier)
        
    def contour_identifier(self, contourlist):
        now = rospy.get_time()
        # keep track of which new objects have been "taken"
        contours_accounted_for = []
        
        update_dict = {}
        
        def update_tracked_object(tracked_object, measurement, contourlist):
            if measurement is None:
                xhat, P, K = tracked_object['kalmanfilter'].update( None ) # run kalman filter
                m = np.matrix([np.nan for i in range( tracked_object['measurement'].shape[0] ) ]).T
            else:
                xhat, P, K = tracked_object['kalmanfilter'].update( tracked_object['measurement'][:,-1] ) # run kalman filter
                tracked_object['measurement'] = np.hstack( (tracked_object['measurement'], measurement) ) # add object's data to the tracked object
            tracked_object['frames'].append(contourlist.header.seq)
            tracked_object['timestamp'].append(contourlist.header.stamp)
            
            tracked_object['state'] = np.hstack( (tracked_object['state'], xhat) )
        
        # iterate through objects first
        # get order of persistence
        objid_in_order_of_persistance = []
        if len(self.tracked_objects.keys()) > 0:
            persistance = []
            objids = []
            for objid, tracked_object in self.tracked_objects.items():
                 persistance.append(len(tracked_object['frames']))
                 objids.append(objid)
            order = np.argsort(persistance)[::-1]
            objid_in_order_of_persistance = [objids[o] for o in order]
        #print len(objid_in_order_of_persistance), objid_in_order_of_persistance
        
        # loop through objects, in order of persistance, so longest existing objects get priority on new measurements        
        for objid in objid_in_order_of_persistance:
            tracked_object = self.tracked_objects[objid]
            tracked_object_state_estimate = tracked_object['kalmanfilter'].xhat_apriori  # extract estimate of current position based on Kalman model
            tracked_object_covariance = np.linalg.norm(tracked_object['kalmanfilter'].P_apriori.diagonal())  # extract current covariance based on Kalman model 
            
            # loop through contours and find best match, provided it is within the range of covariances allowed
            best_contour = None
            best_error = None
            errors = []
            for c, contour in enumerate(contourlist.contours):
                if c in contours_accounted_for:
                    continue
                if contour.area < self.min_size:
                    continue
                measurement = np.matrix([contour.x, contour.y, 0, contour.area, contour.angle]).T
                error = np.linalg.norm( (measurement.T - (tracked_object['kalmanfilter'].H*tracked_object_state_estimate).T)*self.association_matrix )
                errors.append(error)
                if error < 50*np.sqrt(tracked_object_covariance):
                    if best_contour is None:
                        best_contour = c
                        best_error = error
                    else:
                        if error < best_error:
                            best_contour = c
                            best_error = error
                            
            if best_contour is not None:
                contour = contourlist.contours[best_contour]
                measurement = np.matrix([contour.x, contour.y, 0, contour.area, contour.angle]).T
                update_tracked_object(tracked_object, measurement, contourlist)
                contours_accounted_for.append(c)
            
        # any unnaccounted for objects should spawn new objects
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
                            'frames':       [contour.header.seq],
                            'kalmanfilter': Kalman.DiscreteKalmanFilter(x0      = obj_state, 
                                                                        P0      = self.kalman_parameters.P0, 
                                                                        phi     = self.kalman_parameters.phi, 
                                                                        gamma   = self.kalman_parameters.gamma, 
                                                                        H       = self.kalman_parameters.H, 
                                                                        Q       = self.kalman_parameters.Q, 
                                                                        R       = self.kalman_parameters.R, 
                                                                        gammaW  = self.kalman_parameters.gammaW)
                          }
                self.tracked_objects.setdefault(new_obj['objid'], new_obj)
                self.current_objid += 1
                
        # propagate unmatched objects
        for objid, tracked_object in self.tracked_objects.items():
            if tracked_object['frames'][-1] != contourlist.header.seq:
                update_tracked_object(tracked_object, None, contourlist)
        
        # make sure we don't get too many objects - delete the oldest ones, and ones with high covariances
        objects_to_destroy = []
        if len(objid_in_order_of_persistance) > self.max_tracked_objects:
            for objid in objid_in_order_of_persistance[self.max_tracked_objects:]:
                objects_to_destroy.append(objid)
            #print 'destroying: ', len(objects_to_destroy)
            
        for objid, tracked_object in self.tracked_objects.items():
            if np.linalg.norm( tracked_object['kalmanfilter'].P ) > self.max_covariance:
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
            t = rospy.Time.now()
            for objid in objid_in_order_of_persistance:
                if objid not in objects_to_destroy:
                    tracked_object = self.tracked_objects[objid]
                    data = Trackedobject()
                    data.header  = Header(stamp=t)
                    p = np.array( tracked_object['state'][tracked_object['statenames']['position'],-1] ).flatten().tolist()
                    v = np.array( tracked_object['state'][tracked_object['statenames']['velocity'],-1] ).flatten().tolist()
                    data.position       = Point( p[0], p[1], p[2] )
                    data.velocity       = Vector3( v[0], v[1], v[2] )
                    data.angle          = tracked_object['state'][tracked_object['statenames']['angle'],-1]
                    data.size           = tracked_object['state'][tracked_object['statenames']['size'],-1]#np.linalg.norm(tracked_object['kalmanfilter'].P.diagonal())
                    data.covariance     = np.linalg.norm(tracked_object['kalmanfilter'].P.diagonal()[0:2]) # position covariance only
                    data.objid          = tracked_object['objid']
                    data.persistence    = len(tracked_object['frames'])
                    object_info_to_publish.append(data)
            header = Header(stamp=t)
            self.pubTrackedObjects.publish( Trackedobjectlist(header=header, tracked_objects=object_info_to_publish) )
    
    def main(self):
        rospy.spin()
                
                
if __name__ == '__main__':
    
    ### Define kalman filter properties ########
    kalman_parameters = Struct()
    kalman_parameters.phi = np.matrix([  [1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                         [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                         [0, 0, 1, 1, 0, 0, 0, 0, 0, 0],
                                         [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                                         [0, 0, 0, 0, 1, 1, 0, 0, 0, 0],
                                         [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                                         [0, 0, 0, 0, 0, 0, 1, 1, 0, 0],
                                         [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                                         [0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
                                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])
    kalman_parameters.H   = np.matrix([  [1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                         [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                                         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                                         [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                                         [0, 0, 0, 0, 0, 0, 0, 0, 1, 0]])
    kalman_parameters.P0  = 10*np.eye(10)
    kalman_parameters.Q   = 1*np.matrix(np.eye(10))
    kalman_parameters.R   = .0001*np.matrix(np.eye(5))
    
    kalman_parameters.gamma  = None
    kalman_parameters.gammaW = None
    #############################################
    
    data_associator = DataAssociator(kalman_parameters)
    data_associator.main()
