#!/usr/bin/env python
'''
Republish a single object selected based on persistance.
Use standard ROS message types (Float32MultiArray for now)
'''
from __future__ import division
from optparse import OptionParser
import roslib
import rospy
import numpy as np

from std_msgs.msg import Float32MultiArray
from multi_tracker.msg import Trackedobject, Trackedobjectlist

            
# The main tracking class, a ROS node
class PrefObjPicker:
    def __init__(self, nodenum, rate):
        '''
        Default image_topic for:
            Basler ace cameras with camera_aravis driver: camera/image_raw
            Pt Grey Firefly cameras with pt grey driver : camera/image_mono
        '''
        # default parameters (parameter server overides them)
        self.params = { 'min_persistence'   : 10,
                        }
        for parameter, value in self.params.items():
            try:
                p = '/multi_tracker/' + nodenum + '/prefobj/' + parameter
                self.params[parameter] = rospy.get_param(p)
            except:
                pass
                
        # initialize the node
        rospy.init_node('prefobj_' + nodenum)
        self.nodename = rospy.get_name().rstrip('/')
        self.nodenum = nodenum

        self.rate = rate
        self.msg = None

        self.prefObjId = None
        self.subTrackedObjects = rospy.Subscriber('/multi_tracker/' + nodenum + '/tracked_objects', Trackedobjectlist, self.tracked_object_callback)
        self.pubPrefObj = rospy.Publisher('/multi_tracker/' + nodenum + '/prefobj', Float32MultiArray, queue_size=3)
        
    def tracked_object_callback(self, tracked_objects):
        
        obj_ids = []
        persistances = []
        index = []
        
        i = -1
        for tracked_object in tracked_objects.tracked_objects:
            i += 1
            if tracked_object.persistence > self.params['min_persistence']:
                persistances.append(tracked_object.persistence)
                obj_ids.append(tracked_object.objid)
                index.append(i)
                
        if len(persistances) > 0:
            p = np.argmax(persistances)
            self.prefObjId = obj_ids[p]
            
            tracked_object = tracked_objects.tracked_objects[i]
            
            self.msg = Float32MultiArray()
            self.msg.data = [tracked_object.objid, 
                                tracked_object.position.x,
                                tracked_object.position.y,
                                tracked_object.velocity.x,
                                tracked_object.velocity.y]
            
            if self.rate == 0:
                self.pubPrefObj.publish(self.msg)
        else:
            self.msg = None

    def Main(self):
        if self.rate == 0:
            while (not rospy.is_shutdown()):
                rospy.spin()
        else:
            rate = rospy.Rate(self.rate) # 10hz
            while not rospy.is_shutdown():
                if self.msg is not None:
                    self.pubPrefObj.publish(self.msg)
                    rate.sleep()

#####################################################################################################
    
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--nodenum", type="str", dest="nodenum", default='1',
                        help="node number, for example, if running multiple tracker instances on one computer")
    parser.add_option("--rate", type="float", dest="rate", default='0',
                        help="rate at which to (re)publish preferred object location. default=0 means continuous.")
    (options, args) = parser.parse_args()
    
    prefobjpicker = PrefObjPicker(options.nodenum, options.rate)
    prefobjpicker.Main()
