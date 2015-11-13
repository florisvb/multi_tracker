import matplotlib.pyplot as plt
import numpy as np

def get_bins_from_backgroundimage(backgroundimage):
    if type(backgroundimage) is str:
        backgroundimage = plt.imread(backgroundimage)
    binsx = np.arange(0, backgroundimage.shape[1]+1, 1)
    binsy = np.arange(0, backgroundimage.shape[0]+1, 1)
    return binsx, binsy
    
def get_heatmap(pd, binsx, binsy, position_x='position_x', position_y='position_y', position_z='position_z', position_z_slice=None):
    if position_z_slice is not None:
        pd_subset = pd[ (pd[position_z]>position_z_slice[0]) & (pd[position_z]<position_z_slice[1])]
    else:
        pd_subset = pd
    x = pd_subset[position_x].values
    y = pd_subset[position_y].values
    h, xedges, yedges = np.histogram2d(x,y,bins=[binsx,binsy])
    return h
    
def plot_heatmap(pd, binsx, binsy, ax=None, vmin=0, vmax=None, logcolorscale=False, position_x='position_x',position_y='position_y', position_z='position_z', position_z_slice=None):   
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111)
        
    heatmap = get_heatmap(pd, binsx, binsy, position_x, position_y, position_z, position_z_slice)
    if logcolorscale:
        heatmap = np.log(heatmap)
    if vmax is None:
        vmax = 0.1*np.max(heatmap)
    ax.imshow(heatmap.T, cmap=plt.get_cmap('hot'), vmin=vmin, vmax=vmax, origin='lower', extent=[binsx[0],binsx[-1],binsy[0],binsy[-1]])
    
    return ax
    
def plot_trajectories(pd, binsx, binsy, backgroundimage=None, ax=None, position_x='position_x',position_y='position_y', position_z='position_z', position_z_slice=None):
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111)
        
    heatmap = get_heatmap(pd, binsx, binsy, position_x, position_y, position_z, position_z_slice)
    heatmap_binary = np.sign(heatmap).astype(bool)
    masked_heatmap_binary = np.ma.masked_array(heatmap_binary, mask=np.invert(heatmap_binary))
    
    if backgroundimage is not None:
        if type(backgroundimage) is str:
            backgroundimage = plt.imread(backgroundimage)
        ax.imshow(backgroundimage, cmap=plt.get_cmap('gray'), extent=[binsx[0], binsx[-1], binsy[0], binsy[-1]])
        
    ax.imshow(masked_heatmap_binary.T, vmin = 0, vmax = 0.01, extent=[binsx[0], binsx[-1], binsy[0], binsy[-1]])
    
    ax.set_xlim(binsx[0], binsx[-1])
    ax.set_ylim(binsy[0], binsy[-1])
    ax.set_frame_on(False)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_aspect('equal')
    

