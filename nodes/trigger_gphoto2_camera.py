#!/usr/bin/env python
'''
'''
from optparse import OptionParser
import roslib
import rospy
import os
import time

from std_msgs.msg import Float32MultiArray

'''
sudo apt-get install libgphoto2-dev
sudo pip install -v gphoto2 (takes a while, be patient)
'''
import gphoto2 as gp
            
# The main tracking class, a ROS node
class GPhotoCamera:
    def __init__(self, nodenum, topic):
        # where to save photos
        home_directory = os.path.expanduser( rospy.get_param('/multi_tracker/' + nodenum + '/data_directory') )
        experiment_basename = rospy.get_param('/multi_tracker/' + nodenum + '/experiment_basename', 'none')
        if experiment_basename == 'none':
            experiment_basename = time.strftime("%Y%m%d_%H%M%S_N" + nodenum, time.localtime())
        directory_name = experiment_basename + '_gphoto2'
        self.destination = os.path.join(home_directory, directory_name)
        if os.path.exists(self.destination):
            pass
        else:
            os.mkdir(self.destination)

        # initialize the node
        rospy.init_node('gphoto2_' + nodenum)
        self.nodename = rospy.get_name().rstrip('/')
        self.nodenum = nodenum

        #gp.check_result(gp.use_python_logging())
        self.context = gp.gp_context_new()
        self.camera = gp.check_result(gp.gp_camera_new())
        gp.check_result(gp.gp_camera_init(self.camera, self.context))
        self.synchronize_camera_timestamp()
        
        self.subTrackedObjects = rospy.Subscriber('/multi_tracker/' + nodenum + '/' + topic, Float32MultiArray, self.gphoto_callback)
        
    def synchronize_camera_timestamp(self):
        def set_datetime(config):
            OK, date_config = gp.gp_widget_get_child_by_name(config, 'datetime')
            if OK >= gp.GP_OK:
                widget_type = gp.check_result(gp.gp_widget_get_type(date_config))
                if widget_type == gp.GP_WIDGET_DATE:
                    now = int(time.time())
                    gp.check_result(gp.gp_widget_set_value(date_config, now))
                else:
                    now = time.strftime('%Y-%m-%d %H:%M:%S')
                    gp.check_result(gp.gp_widget_set_value(date_config, now))
                return True
            return False

        # get configuration tree
        config = gp.check_result(gp.gp_camera_get_config(self.camera, self.context))
        # find the date/time setting config item and set it
        if set_datetime(config):
            # apply the changed config
            gp.check_result(gp.gp_camera_set_config(self.camera, config, self.context))
        else:
            print('Could not set date & time')
        # clean up
        gp.check_result(gp.gp_camera_exit(self.camera, self.context))
        return 0



    def gphoto_callback(self, msg):

        file_path = gp.check_result(gp.gp_camera_capture(
            self.camera, gp.GP_CAPTURE_IMAGE, self.context))
        print('Camera file path: {0}/{1}'.format(file_path.folder, file_path.name))

        print('Captured image')
        t = rospy.Time.now()
        time_base = time.strftime("%Y%m%d_%H%M%S_N" + self.nodenum, time.localtime())
        print time_base
        name = time_base + '_' + str(t.secs) + '_' + str(t.nsecs) + '.jpg'
        target = os.path.join(self.destination, name)

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
    parser.add_option("--topic", type="str", dest="topic", default='prefobj',
                        help="topic name (will be appended to /multi_tracker/N/). Defaults to the prefobj topic.")
    (options, args) = parser.parse_args()
    
    gphotocamera = GPhotoCamera(options.nodenum, options.topic)
    gphotocamera.Main()
