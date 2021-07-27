import sys
version_info = sys.version_info
python_version = sys.version_info[0]

import multi_tracker_analysis.data_slicing as data_slicing
import multi_tracker_analysis.read_hdf5_file_to_pandas as read_hdf5_file_to_pandas
import multi_tracker_analysis.plot as plot
import multi_tracker_analysis.estimate_R_and_Q as estimate_R_and_Q
import multi_tracker_analysis.Kalman as Kalman
import multi_tracker_analysis.trajectory_analysis as trajectory_analysis

if python_version == 2:
    import multi_tracker_analysis.bag2hdf5 as bag2hdf5
    import multi_tracker_analysis.dvbag_to_pandas_reader as dvbag_to_pandas_reader
else:
    print('Cannot import ROS related packages in python 3')