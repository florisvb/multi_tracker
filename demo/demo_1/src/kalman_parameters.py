import numpy as np

### Define kalman filter properties ########
phi = np.matrix([                    [1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                     [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                     [0, 0, 1, 1, 0, 0, 0, 0, 0, 0],
                                     [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                                     [0, 0, 0, 0, 1, 1, 0, 0, 0, 0],
                                     [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                                     [0, 0, 0, 0, 0, 0, 1, 1, 0, 0],
                                     [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                                     [0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
                                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])

H   = np.matrix([                    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                     [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                                     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                                     [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                                     [0, 0, 0, 0, 0, 0, 0, 0, 1, 0]])
P0  = 10*np.eye(10)
Q   = .2*np.matrix(np.eye(10))
R   = 50*np.matrix(np.eye(5))

gamma  = None
gammaW = None

max_covariance = 30
max_velocity = 10

association_matrix = np.matrix([[1,1,0,0,0]], dtype=float).T
association_matrix /= np.linalg.norm(association_matrix)
