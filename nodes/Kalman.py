import numpy as np
import scipy.stats
import matplotlib.pyplot as plt

# class for running the discrete kalman filter algorithm from table 3.1
class DiscreteKalmanFilter(object):
    def __init__(self, x0=None, P0=None, phi=None, gamma=None, H=None, Q=None, R=None, gammaW=None):
        '''
        Discrete Kalman Filter
        
        nstates ------- number of states
        nmeasurements - number of measurements
        x0  ----------- initial state
        P0  ----------- initial covariance
        phi    -------- drift dynamics model (e.g. A in xdot = Ax + Bu)  
        gamma  -------- control dynamics model (e.g. B in xdot = Ax + Bu)
        H  ------------ observation model, maps states to observations (e.g. C in y = Cx + Du)
        Q  ------------ state covariance
        R  ------------ measurement covariance
        gammaW  -------
            
        '''
        
        # initialize variables, to make class more user friendly
        assert phi is not None, "need to define phi"
        assert H is not None, "need to define H"
        
        self.nstates = phi.shape[0]
        self.nmeasurements = H.shape[0]
    
        if gamma is None:
            ncontrols = 1
            gamma = np.matrix(np.zeros([self.nstates, ncontrols]))
        if Q is None:
            Q = np.matrix(np.eye(self.nstates))
        if R is None:
            R = np.matrix(np.eye(self.nmeasurements))
        if gammaW is None:
            gammaW = np.matrix(np.eye(self.nstates))
        if x0 is None:
            x0 = np.ones([self.nstates, 1])
        if P0 is None:
            P0 = np.matrix(np.eye(self.nstates))

        # save variables for later use
        self.xhat_apriori = x0
        self.xhat = x0
        self.P_apriori = P0
        self.P = P0
        self.P_aposteriori = P0
        self.phi = phi
        self.gamma = gamma
        self.Q = Q
        self.R = R
        self.H = H
        self.gammaW = gammaW
        
    # the meat of the algorithm
    def update(self, measurement=None, control=None):
    
        if control is None:
            control = np.matrix(np.zeros([1, 1]))
        
        # get apriori terms from memory
        xhat_apriori = self.xhat_apriori
        P_apriori = self.P_apriori
        phi = self.phi
        gamma = self.gamma
        Q = self.Q
        R = self.R
        H = self.H
        gammaW = self.gammaW
        
        # calculate kalman gain
        K = P_apriori*H.T*(H*P_apriori*H.T+R).I

        # update step
        if measurement is not None:
            xhat_aposteriori = xhat_apriori + K*(measurement - H*xhat_apriori)
            I = np.matrix(np.eye(self.nstates))
            P_aposteriori = (I-K*H)*P_apriori
        else:
            xhat_aposteriori = xhat_apriori
            P_aposteriori = P_apriori
            
        # propagate step
        xhat_new_apriori = phi*xhat_aposteriori + gamma*control
        P_new_apriori = phi*P_aposteriori*phi.T + gammaW*Q*gammaW.T
        
        # save new terms
        self.xhat_apriori = xhat_new_apriori
        self.P_apriori = P_new_apriori
        self.xhat = xhat_aposteriori
        self.P = P_aposteriori
        
        # return the current aposteriori estimates
        return self.xhat, self.P, K
        
        
if __name__ == '__main__':
    
    # Generate synthetic data
    time_points = np.linspace(0, 20, 500)
    
    clean_pos_x = np.sin(time_points)
    clean_pos_y = np.sin(time_points*3)
    
    dynamics_noise = 1e-6
    observation_noise = 1e-1
    v = scipy.stats.norm(0,dynamics_noise) # noise
    w = scipy.stats.norm(0,observation_noise) # noise
    noisy_pos_x = clean_pos_x + v.rvs(len(clean_pos_x))
    noisy_pos_y = clean_pos_y + v.rvs(len(clean_pos_y))
    
    # Initialize kalman filter
    # Note: number of states = 4 (2 position states, 2 velocity states)
    # State vector: [[pos_x], [vel_x], [pos_y], [vel_y]] (note shape will be a column vector)
    
    # Drift dynamics (constant velocity model)
    phi = np.matrix([[1, 1, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, 1],
                     [0, 0, 0, 1]])
                     
    # Observation dynamics (observe position and velocity)
    H   = np.matrix([[1, 0, 0, 0],
                     [0, 0, 1, 0]])
                     
    # Initial covariance
    P0  = 10*np.eye(4)
    
    # Dynamics covariance
    Q   = dynamics_noise**.5*np.matrix(np.eye(4))
    
    # Observation covariance
    R   = observation_noise**.5*np.matrix(np.eye(2))
    
    # Initial state
    x0 = np.matrix([noisy_pos_x[0], 0, noisy_pos_y[0], 0]).T

    kalman_filter = DiscreteKalmanFilter(   x0=x0, 
                                            P0=P0, 
                                            phi=phi, 
                                            gamma=None, 
                                            H=H, 
                                            Q=Q, 
                                            R=R, 
                                            gammaW=None)    
    
    # Run the filter
    
    # Initialize filtered data arrays
    filtered_pos_x = np.zeros_like(noisy_pos_x)
    filtered_vel_x = np.zeros_like(noisy_pos_x)
    filtered_pos_y = np.zeros_like(noisy_pos_y)
    filtered_vel_y = np.zeros_like(noisy_pos_y)
    observation_x = np.zeros_like(noisy_pos_x)
    observation_y = np.zeros_like(noisy_pos_y)
    covariance = np.zeros_like(noisy_pos_y)
    
    # Iterate through the data
    for i in range(1, len(noisy_pos_x)):
        observation = np.matrix([noisy_pos_x[i]+w.rvs(), noisy_pos_y[i]+w.rvs()]).T
        xhat, P, K = kalman_filter.update(observation)
        filtered_pos_x[i] = xhat[0,0]
        filtered_vel_x[i] = xhat[1,0]
        filtered_pos_y[i] = xhat[2,0]
        filtered_vel_y[i] = xhat[3,0]
        observation_x[i] = observation[0]
        observation_y[i] = observation[1]
        covariance[i] = np.linalg.norm(P.diagonal())
        
    # Plot the results
    # Black = real data, '*' is noisy data
    # Red = filtered data
    fig = plt.figure()
    ax_x = fig.add_subplot(311)
    ax_y = fig.add_subplot(312)
    ax_cov = fig.add_subplot(313)
    
    ax_x.plot(time_points, clean_pos_x, 'black')
    ax_x.plot(time_points, noisy_pos_x, '*', color='black')
    ax_x.plot(time_points, observation_x, '*', color='green')
    ax_x.plot(time_points, filtered_pos_x, color='red')
    ax_x.set_xlabel('time')
    ax_x.set_ylabel('x position')
    
    ax_y.plot(time_points, clean_pos_y, 'black')
    ax_y.plot(time_points, noisy_pos_y, '*', color='black')
    ax_y.plot(time_points, observation_y, '*', color='green')
    ax_y.plot(time_points, filtered_pos_y, color='red')
    ax_y.set_xlabel('time')
    ax_y.set_ylabel('y position')
    
    ax_cov.plot(time_points, 3*np.sqrt(covariance), 'blue')
    ax_cov.plot(time_points, np.abs(filtered_pos_x - noisy_pos_x), 'red')
    ax_cov.set_xlabel('time')
    ax_cov.set_ylabel('covariance')
