
import read_hdf5_file_to_pandas
import find_flies_in_image_directory
import os
import pickle
import cv2
import numpy as np
import numpy.linalg
import matplotlib.pyplot as plt
import scipy.optimize

def load_gphoto2_flies(path):
    config = read_hdf5_file_to_pandas.load_config_from_path(path)
    s = config.identifiercode + '_' + 'gphoto2'
    gphoto2directory = os.path.join(config.path, s)
    fname = os.path.join(gphoto2directory, 'fly_ellipses.pickle')
    if not os.path.exists(fname):
        find_flies_in_image_directory.find_flies_in_images(gphoto2directory)
    f = open(fname)
    gphoto2_flies = pickle.load(f)
    f.close()
    return gphoto2_flies

def __choose_frames_with_a_single_fly(gphoto2_flies):
    unique_gphoto2_flies = {}
    for filename, flies in gphoto2_flies.items():
        if len(flies) == 1:
            unique_gphoto2_flies[filename] = flies[0]
    return unique_gphoto2_flies

def __find_homologous_flies(unique_gphoto2_flies, tracking_pd_frame, delay=0):
    points_2d_gphoto2 = []
    points_2d_tracker = []

    for filename, fly in unique_gphoto2_flies.items():
        basename = os.path.basename(filename).split('.')[0]
        secs = float(basename.split('_')[-2])
        nsecs = float(basename.split('_')[-1])
        t = secs + nsecs*1e-9 - delay

        index = np.argmin( np.abs(tracking_pd_frame.time_epoch - t) )
        #frame = tracking_pd_frame.loc[index].frames # should be redundant
        #print frame, index
        frame_data = tracking_pd_frame[tracking_pd_frame.index==index]
        if len(frame_data) == 1: # also only 1 fly
            points_2d_tracker.append( [frame_data.position_x.values[0], frame_data.position_y.values[0]] )
            points_2d_gphoto2.append( [fly[0][0], fly[0][1]] )

    return points_2d_tracker, points_2d_gphoto2 # lists

def __calc_homography_from_tracker_to_gphoto2(points_2d_tracker, points_2d_gphoto2):
    M = cv2.findHomography(np.array(points_2d_tracker), np.array(points_2d_gphoto2))
    Hm = M[0]
    return Hm

def reproject_tracker_point_onto_gphoto2(points_2d_tracker, points_2d_gphoto2, Hm):
    h11, h12, h13, h21, h22, h23, h31, h32, h33 = np.ravel(Hm)
    points_2d_reprojected = []
    for point in points_2d_tracker:
        x, y = point
        x_repr = (h11*x + h12*y + h13)/(h31*x + h32*y + h33)
        y_repr = (h21*x + h22*y + h23)/(h31*x + h32*y + h33)
        points_2d_reprojected.append([x_repr, y_repr])
    return points_2d_reprojected

def __calc_reprojection_errors_from_tracker_to_gphoto2(points_2d_tracker, points_2d_gphoto2):
    Hm = __calc_homography_from_tracker_to_gphoto2(points_2d_tracker, points_2d_gphoto2)
    points_2d_reprojected = reproject_tracker_point_onto_gphoto2(points_2d_tracker, points_2d_gphoto2, Hm)
    return np.linalg.norm( np.array(points_2d_gphoto2) - np.array(points_2d_reprojected))

def __calc_reprojection_errors_for_delay(delay, unique_gphoto2_flies, tracking_pd_frame):
    points_2d_tracker, points_2d_gphoto2 = __find_homologous_flies(unique_gphoto2_flies, tracking_pd_frame, delay=delay)
    return __calc_reprojection_errors_from_tracker_to_gphoto2(points_2d_tracker, points_2d_gphoto2)

def __find_optimal_delay(unique_gphoto2_flies, tracking_pd_frame, delay=1):
    delay_opt = scipy.optimize.fmin(__calc_reprojection_errors_for_delay, delay, args=(unique_gphoto2_flies, tracking_pd_frame))
    return delay_opt[0]

def get_optimal_gphoto2_homography(path, delay_guess=1, save=True, recalculate=False):
    config = read_hdf5_file_to_pandas.load_config_from_path(path)
    s = config.identifiercode + '_' + 'gphoto2'
    gphoto2directory = os.path.join(config.path, s)
    fname = os.path.join(gphoto2directory, 'tracker_to_gphoto2_homography_matrix.pickle')

    if not os.path.exists(fname) or recalculate:
        data_filename = read_hdf5_file_to_pandas.get_filename(path, 'trackedobjects.hdf5')
        tracking_pd_frame, config = read_hdf5_file_to_pandas.load_and_preprocess_data(data_filename)
        gphoto2_flies = load_gphoto2_flies(path)
        unique_gphoto2_flies = __choose_frames_with_a_single_fly(gphoto2_flies)
        delay_opt = __find_optimal_delay(unique_gphoto2_flies, tracking_pd_frame, delay=delay_guess)
        points_2d_tracker, points_2d_gphoto2 = __find_homologous_flies(unique_gphoto2_flies, tracking_pd_frame, delay=delay_opt)
        Hm = __calc_homography_from_tracker_to_gphoto2(points_2d_tracker, points_2d_gphoto2)
        f = open(fname, 'w')
        pickle.dump(Hm, f)
        f.close()
    else:
        f = open(fname)
        Hm = pickle.load(f)
        f.close()

    return Hm

### Analysis and testing functions

def __plot_reprojected_points(points_2d_tracker, points_2d_gphoto2, Hm):
    points_2d_reprojected = reproject_tracker_point_onto_gphoto2(points_2d_tracker, points_2d_gphoto2, Hm)

    fig = plt.figure()
    ax = fig.add_subplot(111)

    ax.plot(np.array(points_2d_gphoto2)[:,0], np.array(points_2d_gphoto2)[:,1], 'o', markersize=10, color='green')
    ax.plot(np.array(points_2d_reprojected)[:,0], np.array(points_2d_reprojected)[:,1], 'o', markersize=3, color='red')

def __get_gphoto2_projection_of_tracker_point(point_2d_tracker, Hm):
    point_2d_tracker.append(1)

def __get_test_data():

    path = '/home/riffelluser/demo/demo_1/gphoto_test_2'

    data_filename = read_hdf5_file_to_pandas.get_filename(path, 'trackedobjects.hdf5')
    tracking_pd_frame, config = read_hdf5_file_to_pandas.load_and_preprocess_data(data_filename)

    gphoto2_flies = load_gphoto2_flies(path)


    unique_gphoto2_flies = __choose_frames_with_a_single_fly(gphoto2_flies)

    return unique_gphoto2_flies, tracking_pd_frame  

def __test(delay=0):

    path = '/home/riffelluser/demo/demo_1/gphoto_test_2'

    data_filename = read_hdf5_file_to_pandas.get_filename(path, 'trackedobjects.hdf5')
    tracking_pd_frame, config = read_hdf5_file_to_pandas.load_and_preprocess_data(data_filename)

    gphoto2_flies = load_gphoto2_flies(path)


    unique_gphoto2_flies = __choose_frames_with_a_single_fly(gphoto2_flies)
    points_2d_tracker, points_2d_gphoto2 = __find_homologous_flies(unique_gphoto2_flies, tracking_pd_frame, delay=delay)

    Hm = __calc_homography_from_tracker_to_gphoto2(points_2d_tracker, points_2d_gphoto2)

    points_2d_reprojected = reproject_tracker_point_onto_gphoto2(points_2d_tracker, points_2d_gphoto2, Hm)

    print Hm
    print points_2d_reprojected
    print
    print np.linalg.norm( np.array(points_2d_gphoto2) - np.array(points_2d_reprojected))

    __plot_reprojected_points(points_2d_tracker, points_2d_gphoto2, Hm)

    __plot_reprojected_points(points_2d_gphoto2, points_2d_tracker, numpy.linalg.inv(Hm))