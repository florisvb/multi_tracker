Multi tracker
============

Multi tracker is a basic ROS package for tracking multiple objects in 2D.

Installing
============

Multi tracker is a ROS catkin package. Put the package in your catkin_workspace/src, and run catkin_make from the catkin_workspace.

Running
============

*rosrun multi_tracker tracker.py*

Will run a ROS node that extracts the outer most contours of a background subtracted, thresholded image.

*rosrun multi_tracker data_association.py*

Will run a ROS node that uses a simple discrete Kalman filter to connect current observations to previous ones, allowing the tracking of multiple objects. These results are published to /multi_tracker/tracked_objects 

Necessary components
============

ROS (tested with hydro, on Ubuntu 12.04): http://wiki.ros.org/hydro

A camera. Options (tested) include:

Basler ACE (GigE), using the aravis driver, and camera_aravis for ROS
https://github.com/ssafarik/camera_aravis
rosrun camera_aravis camnode
default image: /camera/image_raw

Pt Grey Firefly (USB), using the point-grey
http://wiki.ros.org/pointgrey_camera_driver
rosrun pointgrey_camera_driver camera_node
default image: /camera/image_mono

Useful commands:
rosrun rqt_reconfigure rqt_reconfigure

