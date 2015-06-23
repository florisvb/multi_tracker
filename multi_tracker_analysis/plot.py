import matplotlib
import matplotlib.pyplot as plt
import numpy as np

import data_slicing
import plotting_preferences

def plot_trajectories(data, keys, image_filename=None, show_keys=False):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    
    if image_filename is not None:
        image = plt.imread(image_filename)
        ax.imshow(image, cmap='gray')
    
    for key in keys:
        trajec = data[key]
        ax.plot(trajec.position[:,0], trajec.position[:,1])
        if show_keys:
            ax.text(trajec.position[0,0], trajec.position[0,1], key, {'fontsize': 7})
    ax.set_aspect('equal')
    
def plot_trajectories_of_length_greater_than(data, length, keys=None):
    if keys is None:
        keys = keys = data.keys()
    keys_long_enough = data_slicing.get_keys_of_length_greater_than(data, length, keys=keys)
    plot_trajectories(data, keys_long_enough)
    
##################################################################################################
# Heatmaps

def get_bins_and_blank_array(xlim, ylim, resolution=0.02):
    binsx = np.arange(xlim[0],xlim[-1],resolution)
    binsy = np.arange(ylim[0],ylim[-1],resolution)
    arr = np.zeros([len(binsy)-1, len(binsx)-1])
    return binsx, binsy, arr
    
def get_heatmap_array(data, xlim, ylim, resolution, keys=None, axes=[0,1]):
    if keys is None:
        keys = keys = data.keys()
    
    binsx, binsy, arr = get_bins_and_blank_array(xlim, ylim, resolution)
    
    for key in keys:
        trajec = data[key]
        x = trajec.position[:,axes[0]]
        y = trajec.position[:,axes[1]]
        hist, bx, by = np.histogram2d(x,y,bins=(binsx, binsy))
        arr += hist.T
               
    return binsx, binsy, arr
    
def plot_heatmap(data, xlim, ylim, resolution, keys=None, axes=[0,1], norm=[0,0.03]):
    '''
    norm - fraction of all frames to use as max and min of heatmap colorscheme
    '''

    binsx, binsy, arr = get_heatmap_array(data, xlim, ylim, resolution, keys, axes)
            
    fig = plt.figure()
    ax = fig.add_subplot(111)
    
    all_frames = np.sum(arr)
    vmin = all_frames*norm[0]
    vmax = all_frames*norm[-1]
    colornorm = matplotlib.colors.Normalize(vmin, vmax)
    
    ax.imshow(arr, cmap=plotting_preferences.heatmap_colormap, interpolation=plotting_preferences.heatmap_interpolation, norm=colornorm, origin=plotting_preferences.heatmap_origin, extent=[binsx[0], binsx[-1], binsy[0], binsy[-1]])
            
            
##################################################################################################

def plot_histogram_of_sizes(data):
    sizes = []
    for key, trajec in data.items():
        sizes.extend(trajec.size.tolist())
    
    bins = np.linspace(0,200,200)
    plt.hist(sizes, bins=bins)
    
##################################################################################################

def plot_histogram_of_activity(dataset):
    times = []
    for key, trajec in dataset.items():
        times.extend(trajec.time_local.tolist())
    bins = np.linspace(0,24,48)
    plt.hist(times, bins=bins)
    






