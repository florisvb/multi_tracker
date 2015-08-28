import numpy as np
import data_slicing_w
import matplotlib.pyplot as plt
import orchard.orchard_metadata
import os

def plot_trajectories(data, keys=None, bgimg=None):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    for key in keys:
        print key
        trajec = data.trajec(key)
        ax.plot(trajec.position[:,0], trajec.position[:,1])        
    
    if bgimg is not None:
        img = plt.imread(bgimg)
        ax.imshow(img, zorder=-1000, cmap=plt.get_cmap('gray'))
    
    ax.set_frame_on(False)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_aspect('equal')
    
def plot_trajectories_in_framerange(data, framerange, bgimg=None):
    keys = data_slicing_w.get_keys_in_framerange(data, framerange)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    for key in keys:
        print key
        trajec = data.trajec(key)
        indices = np.where( (trajec.frames>framerange[0])*(trajec.frames<framerange[-1]) )[0]
        ax.plot(trajec.position[indices,0], trajec.position[indices,1])        
    
    if bgimg is not None:
        img = plt.imread(bgimg)
        ax.imshow(img, zorder=-1000, cmap=plt.get_cmap('gray'))
    
    ax.set_frame_on(False)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_aspect('equal')

def plot_trajectory_heatmap_in_framerange(config, data, framerange):
    
    indices = np.where( (data.hdf5data['header.frame_id']>framerange[0])*(data.hdf5data['header.frame_id']<framerange[-1]) )
    x = data.hdf5data[indices]['position.x']
    y = data.hdf5data[indices]['position.y']
    
    h, bx, by = np.histogram2d(x, y, bins=[np.arange(0,config.background_image.shape[1]), np.arange(0,config.background_image.shape[0])])
    h_binary = np.sign(h).astype(bool)
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    orchard.orchard_metadata.draw_image(ax, config)
    
    masked_h = np.ma.masked_array(h_binary, mask=np.invert(h_binary))
    
    ax.imshow(masked_h.T, vmin = 0, vmax = 0.01)
    
    config.draw_mpl(ax, data.framestamp_to_timestamp(framerange[0]+int((framerange[-1]-framerange[0])/2.)))
    
    ax.set_xlim(0,config.background_image.shape[1])
    ax.set_ylim(0,config.background_image.shape[0])
    ax.set_frame_on(False)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_aspect('equal')
    
    return fig
    
def plot_trajectory_heatmap_movie(config, data, frameranges, destination):
    plt.close('all')
    
    for i, framerange in enumerate(frameranges):
        print i
        fig = plot_trajectory_heatmap_in_framerange(config, data, framerange)
        fname = str(i).zfill(5) + '.png'
        fname = os.path.join(destination, fname)
        fig.savefig(fname, format='png') 
        plt.close('all')
    
    print 'mencoder \'mf://*.png\' -mf type=png:fps=30 -ovc lavc -lavcopts vcodec=mpeg4 -oac copy -o animation.avi'







    
    
