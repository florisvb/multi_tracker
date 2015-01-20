Multi tracker
============

Multi tracker is a basic ROS package for tracking multiple objects in 2D. It does not handle object-object interactions (merging contours).

The package was built and tested with point grey usb firefly cameras on an Ubuntu (12.04) system, and the demo code is set up to run with those cameras. However, there is no reason that it shouldn't work with other cameras.

Installing
============

Install ROS, if you have not already done so (tested with ros hydro, full desktop install): http://wiki.ros.org/hydro/Installation/Ubuntu

Setup your catkin workspace, if you have not already done so: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
Install the appropriate camera driver, such as: http://wiki.ros.org/pointgrey_camera_driver

Copy the multi tracker package into your catkin_workspace/src, and run catkin_make from the catkin_workspace.

To talk to the camera, you may need a udev rule. There is an example udev rule for a point grey firefly camera in the rules folder. Move this file to /etc/udev/rules.d directory.

To use the analysis tools, from inside multi_tracker, run python ./setup.py install. You may want to do this in a virtual environment: http://docs.python-guide.org/en/latest/dev/virtualenvs/

Overview
============

Parameters - examples are found in the /demo folder
------------

camera_parameters.yaml: specifies key camera parameters, such as framerate, exposure time, etc. These parameter names may be camera brand dependent. The camera parameters can also be specified by running *rosrun rqt_reconfigure rqt_reconfigure* 

tracker_parameters.yaml: specifies various tracking related parameters

data_association_parameters.yaml: specifies various data association related parameters

kalman_parameters.py: specifies the kalman parameters

Nodes
------------

tracker.py: listens to /camera/image_mono (or whatever topic is specified in tracker_parameters.yaml), uses simple background subtraction and contour detection to find the outer most contours of objects in the tracking space. Publishes a list of contours each frame, including x,y position, angle, and area. 

data_association.py: listens to the '/multi_tracker/contours' topic and runs a simple kalman filter to do data association and filtering between subsequent frames. Publishes tracked objects to '/multi_tracker/tracked_objects' 

save_data_to_csv.py: listens to '/multi_tracker/tracked_objects' and saves the data to a csv file. Alternatively, one can use "rosbag record" to record the data in ros format, which can replayed at a later time.


Running
============

Minimal steps to run:

1. copy the /demo folder to your home directory
2. from inside the demo folder, run "roslaunch example.launch"
   This will load all the yaml (parameter) files, and launch the tracker, data_association, and save_data_to_csv nodes.
3. Hit control-c to stop the node (and cease collecting data).
4. from inside the demo folder, run "python ./analyzer_data.py" to analyze the default data.csv file, and make a plot using matplotlib. For this to work, you must have installed multi_tracker_analysis

Now you can try editing some of the contents of the yaml files to change the file structure and tracking parameters.

Necessary components
============

ROS (tested with hydro, on Ubuntu 12.04): http://wiki.ros.org/hydro

A camera. Options (tested) include:

Pt Grey Firefly (USB), using the point-grey
http://wiki.ros.org/pointgrey_camera_driver
