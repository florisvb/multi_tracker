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
Q   = 1*np.matrix(np.eye(10))
Q_nodata   = 1e-100*np.matrix(np.eye(10))
R   = 10*np.matrix(np.eye(5)) # this is a function of scale - pixels vs mm vs meters
R_nodata   = 10000*np.matrix(np.eye(5))

gamma  = None
gammaW = None

max_covariance = 100

association_matrix = np.matrix([[1,1,0,0,0]], dtype=float).T
association_matrix /= np.linalg.norm(association_matrix)
