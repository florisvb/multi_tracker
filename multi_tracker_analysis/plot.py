import matplotlib.pyplot as plt
import data_slicing

def plot_trajectories(data, length):
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    
    keys = data_slicing.get_keys_of_length_greater_than(data, length)
    
    for key in keys:
        trajec = data[key]
        ax.plot(trajec.position[:,0], trajec.position[:,1])
    
    ax.set_aspect('equal')
