import data_slicing
import kalman_parameters
import numpy as np
import sys
sys.path.append('../nodes')
import Kalman
import matplotlib.pyplot as plt

#keys = data_slicing.get_keys_of_length_greater_than(data, 30)


def find_covariance_Q_for_trajectory(trajec, q=1, r=10):

    Q   = q*np.matrix(np.eye(kalman_parameters.phi.shape[0]))
    R   = r*np.matrix(np.eye(5))
    
    KF_nm = Kalman.DiscreteKalmanFilter(x0=np.matrix([trajec.position[0, 0], 0, trajec.position[0, 1], 0, 0, 0, 0, 0, 0, 0]).T, 
                                        P0=kalman_parameters.P0, 
                                        phi=kalman_parameters.phi, 
                                        gamma=None, 
                                        H=kalman_parameters.H, 
                                        Q=Q, 
                                        R=kalman_parameters.R, 
                                        gammaW=None, 
                                        Q_nodata=None, 
                                        R_nodata=None)

    errors = []
    positional_covariances = []
    KF_covariances = []
    
    for frame in range(0,30):
        measurement = np.matrix([ trajec.position[frame, 0], trajec.position[frame, 1], 0, 0, 0]).T
        xhat, P, K = KF_nm.update(measurement)
        estimated_position = np.array([xhat[0,0], xhat[2,0]])
        actual_position = np.array([trajec.position[frame, 0], trajec.position[frame,1]] )
        print estimated_position, actual_position
        error = np.linalg.norm(estimated_position - actual_position)
        errors.append(error)
        cov = error*error
        positional_covariances.append(cov)
        KF_covariances.append( np.linalg.norm(np.array([P[0,0], P[2,2]])) )
        
    for frame in range(30,trajec.length):
        xhat, P, K = KF_nm.update(None)
        estimated_position = np.array([xhat[0,0], xhat[2,0]])
        actual_position = np.array([trajec.position[frame, 0], trajec.position[frame,1]] )
        print estimated_position, actual_position
        error = np.linalg.norm(estimated_position - actual_position)
        errors.append(error)
        cov = error*error
        positional_covariances.append(cov)
        KF_covariances.append( np.linalg.norm(np.array([P[0,0], P[2,2]])) )
    
    return positional_covariances, KF_covariances
    
    
def find_covariance_Q_for_data(data, keys, q=1, r=10):
    
    p = []
    k = []
    
    for key in keys:
        trajec = data[key]
        p_, k_ = find_covariance_Q_for_trajectory(trajec, q, r)
        p.append(p_)
        k.append(k_)

    fig = plt.figure()
    ax = fig.add_subplot(111)
        
    for i, key in enumerate(keys):
        ax.plot(p[i], color='green')
        ax.plot(k[i], color='red')
    
    plt.show()
    
