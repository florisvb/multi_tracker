#!/usr/bin/env python
'''
'''
from optparse import OptionParser
import roslib
import rospy

from std_msgs.msg import Bool
import gphoto2 as gp
import os
            
# The main tracking class, a ROS node
class GPhotoCamera:
    def __init__(self, nodenum):
        # default parameters (parameter server overides them)
        '''
        self.params = { 'min_persistence'   : 10,
                        }
        for parameter, value in self.params.items():
            try:
                p = '/multi_tracker/' + nodenum + '/prefobj/' + parameter
                self.params[parameter] = rospy.get_param(p)
            except:
                pass
        '''

        # initialize the node
        rospy.init_node('gphoto2_' + nodenum)
        self.nodename = rospy.get_name().rstrip('/')
        self.nodenum = nodenum

        #gp.check_result(gp.use_python_logging())
        self.context = gp.gp_context_new()
        self.camera = gp.check_result(gp.gp_camera_new())
        gp.check_result(gp.gp_camera_init(self.camera, self.context))
        
        self.subTrackedObjects = rospy.Subscriber('/multi_tracker/' + nodenum + '/gphoto', Bool, self.gphoto_callback)
        
    def gphoto_callback(self, msg):

        
        print('Capturing image')

        file_path = gp.check_result(gp.gp_camera_capture(
            self.camera, gp.GP_CAPTURE_IMAGE, self.context))
        print('Camera file path: {0}/{1}'.format(file_path.folder, file_path.name))

        target = os.path.join('/home/caveman', file_path.name)

        print('Copying image to', target)
        camera_file = gp.check_result(gp.gp_camera_file_get(
                self.camera, file_path.folder, file_path.name,
                gp.GP_FILE_TYPE_NORMAL, self.context))
        gp.check_result(gp.gp_file_save(camera_file, target))
        #subprocess.call(['xdg-open', target])
        #gp.check_result(gp.gp_camera_exit(self.camera, self.context))

        
    def Main(self):
        while (not rospy.is_shutdown()):
            rospy.spin()

#####################################################################################################
    
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--nodenum", type="str", dest="nodenum", default='1',
                        help="node number, for example, if running multiple tracker instances on one computer")
    (options, args) = parser.parse_args()
    
    gphotocamera = GPhotoCamera(options.nodenum)
    gphotocamera.Main()
