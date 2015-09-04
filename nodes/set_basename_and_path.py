#!/usr/bin/env python
from optparse import OptionParser
import roslib
import rospy
import rosparam
import time

#####################################################################################################
    
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--nodenum", type="str", dest="nodenum", default='1',
                        help="node number, for example, if running multiple tracker instances on one computer")
    (options, args) = parser.parse_args()
    
    experiment_basename = time.strftime("%Y%m%d_%H%M%S_N" + options.nodenum, time.localtime())
    rospy.set_param('/multi_tracker/' + options.nodenum + '/experiment_basename', experiment_basename)
