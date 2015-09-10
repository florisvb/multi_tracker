Multi tracker
============

Multi tracker is a basic ROS package for tracking multiple objects in 2D.

The package was built and tested with point grey usb firefly cameras on an Ubuntu (12.04) system, and Basler GigE cameras using the camera aravis driver on 12.04 and 14.04. However, there is no reason that it shouldn't work with other cameras.

Installing
============

Install ROS, if you have not already done so (tested with ros hydro, full desktop install): http://wiki.ros.org/hydro/Installation/Ubuntu

Setup your catkin workspace, if you have not already done so: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Copy the multi tracker package into your catkin_workspace/src, and run catkin_make from the catkin_workspace. Use "git clone https://github.com/florisvb/multi_tracker.git" for this. 

Cameras
============

Point grey usb camera:

Install the appropriate camera driver, such as: http://wiki.ros.org/pointgrey_camera_driver. To talk to the camera, you may need a udev rule. There is an example udev rule for a point grey firefly camera in the rules folder. Move this file to /etc/udev/rules.d directory.

Basler GigE camera / Camera Aravis:

See aravis_install_notes

Analysis
============

To use the analysis tools: from inside multi_tracker, run python ./setup.py install. You may want to do this in a virtual environment: http://docs.python-guide.org/en/latest/dev/virtualenvs/

Analysis tools currently rely on pandas and hdf5 file formats.

Overview
============

Parameters
------------

(examples are found in the /demo folder)

camera_parameters.yaml: specifies key camera parameters, such as framerate, exposure time, etc. These parameter names may be camera brand dependent. The camera parameters can also be specified by running *rosrun rqt_reconfigure rqt_reconfigure* 

tracker_parameters.yaml: specifies various tracking related parameters

data_association_parameters.yaml: specifies various data association related parameters

kalman_parameters.py: specifies the kalman parameters

Nodes
------------

tracker_simplebuffer.py: listens to /camera/image_mono (or whatever topic is specified in tracker_parameters.yaml), uses simple background subtraction and contour detection to find the outer most contours of objects in the tracking space. Publishes a list of contours each frame, including x,y position, angle, and area. 

data_association.py: listens to the '/multi_tracker/contours' topic and runs a simple kalman filter to do data association and filtering between subsequent frames. Publishes tracked objects to '/multi_tracker/tracked_objects' 

save_data_to_hdf5.py: listens to '/multi_tracker/tracked_objects' and saves the data to a csv file. Alternatively, one can use "rosbag record" to record the data in ros format, which can replayed at a later time.


Running
============

Minimal steps to run:

1. copy the /demo folder to your home directory
2. from inside the demo folder, run "roslaunch example.launch"
   This will load all the yaml (parameter) files, and launch the tracker, data_association, and save_data_to_csv nodes.
3. Hit control-c to stop the node (and cease collecting data).

Now you can try editing some of the contents of the yaml files to change the file structure and tracking parameters.

