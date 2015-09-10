import data_slicing
import numpy as np
import sys
sys.path.append('../nodes')
import Kalman
import matplotlib.pyplot as plt
import imp


def find_covariance_R_given_static_object_data(x):
    '''
    x - list or np.array - time series of measured x position of a static object (e.g. a dead fly)
        ideally data is collected using the same image processing algorithm, arena, etc.
    '''
    if type(x) is list:
        x = np.array(x)
    x_expected = np.tile(np.mean(x), len(x))
    x_centered = x-x_expected 
    x_cov = np.dot(x_centered, x_centered.T)    
    return x_cov

def calc_actual_covariance_for_trajectory_given_q(trajec, kalman_parameter_py_file, q=1, r=10, no_data_after_nth_frame=30):
    '''
    This function propagates kalman filter estimates and covariances with no new information after frame=no_data_after_nth_frame for a given Q=q*I
    The actual covariances of the data are calculated as (cov_i = (xi-xi_expected)*(xi-xi_expected)') 
    These can then be compared to the covariances from the kalman filter
    
    Returns: actual positional covariances, kalman filter positional covariances
    
    Strategy: run the function for various values of q, until the actual and kalman estimated covariances grow at approximately the same rate
    
    trajec - trajectory object from multi_tracker_analysis.read_csv_file_to_python.load_data_as_python_object_from_csv_file
             trajectory should be minimum 2*no_data_after_nth_frame frames long, the longer the better
    '''
    
    kalman_parameters = imp.load_source('kalman_parameters', kalman_parameter_py_file)
    Q   = q*np.matrix(np.eye(kalman_parameters.phi.shape[0]))
    R   = r*np.matrix(np.eye(5))
    
    KF_nm = Kalman.DiscreteKalmanFilter(x0=np.matrix([trajec.position_x.iloc[0], 0, trajec.position_y.iloc[0], 0, 0, 0, 0, 0, 0, 0]).T, 
                                        P0=kalman_parameters.P0, 
                                        phi=kalman_parameters.phi, 
                                        gamma=None, 
                                        H=kalman_parameters.H, 
                                        Q=Q, 
                                        R=kalman_parameters.R, 
                                        gammaW=None,
                                        )

    errors = []
    positional_covariances = []
    KF_covariances = []
    
    for frame in range(0,no_data_after_nth_frame):
        measurement = np.matrix([ trajec.position_x.iloc[frame], trajec.position_y.iloc[frame], 0, 0, 0]).T
        xhat, P, K = KF_nm.update(measurement)
        estimated_position = np.array([xhat[0,0], xhat[2,0]])
        actual_position = np.array([trajec.measurement_x.iloc[frame], trajec.measurement_y.iloc[frame]] )
        error = np.linalg.norm(estimated_position - actual_position)
        errors.append(error)
        cov = error*error
        positional_covariances.append(cov)
        KF_covariances.append( np.linalg.norm(np.array([P[0,0], P[2,2]])) )
        
    for frame in range(no_data_after_nth_frame,len(trajec.measurement_x)):
        xhat, P, K = KF_nm.update(None)
        estimated_position = np.array([xhat[0,0], xhat[2,0]])
        actual_position = np.array([trajec.measurement_x.iloc[frame], trajec.measurement_y.iloc[frame]] )
        error = np.linalg.norm(estimated_position - actual_position)
        errors.append(error)
        cov = error*error
        positional_covariances.append(cov)
        KF_covariances.append( np.linalg.norm(np.array([P[0,0], P[2,2]])) )
    
    return positional_covariances, KF_covariances
    
    
def calc_actual_covariance_for_all_trajecs_given_q(dataset, keys, kalman_parameter_py_file, q=1, r=10, no_data_after_nth_frame=30):
    '''
    Runs the function "calc_actual_covariance_for_trajectory_given_q" on all trajectories associated with the given keys, and plots the resulting covariances
    
    Use "keys = multi_tracker_analysis.data_slicing.get_keys_of_length_greater_than(data, no_data_after_nth_frame*2)" to get the keys
    '''
    p = []
    k = []
    
    good_keys = []
    for key in keys:
        trajec = dataset.trajec(key)
        if len(trajec.measurement_x) < no_data_after_nth_frame*2:
            continue
        p_, k_ = calc_actual_covariance_for_trajectory_given_q(trajec, kalman_parameter_py_file, q, r, no_data_after_nth_frame)
        p.append(p_)
        k.append(k_)
        good_keys.append(key)
        
    fig = plt.figure()
    ax = fig.add_subplot(111)
        
    for i, key in enumerate(good_keys):
        ax.plot(p[i], color='green')
        ax.plot(k[i], color='red')
    
    plt.show()
    
    







    
    
