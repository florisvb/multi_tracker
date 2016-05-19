import matplotlib.pyplot as plt
import numpy as np

def get_bins_from_backgroundimage(backgroundimage, pixel_resolution=1):
    if type(backgroundimage) is str:
        backgroundimage = plt.imread(backgroundimage)
    binsx = np.arange(0, backgroundimage.shape[1]+1, pixel_resolution)
    binsy = np.arange(0, backgroundimage.shape[0]+1, pixel_resolution)
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
    
def get_heatmap_weighted(pd, sample_names, bins, weight_name=None, normed=False):
    sample = [pd[sample_name].values for sample_name in sample_names]
    if weight_name is not None:
        weights = pd[weight_name]
    else:
        weights = None
    h, edges = np.histogramdd(sample,bins,weights=weights, normed=normed)
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

def resampling_statistics_on_heatmaps(h_ons, h_offs, iterations=1000, neff=None):
    '''
    neff - effective N, taking into account clustering. See: http://www.nature.com/neuro/journal/v17/n4/pdf/nn.3648.pdf
    '''
    if neff is None:
        neff = len(h_ons)
    
    # bootstrap actual difference
    actual_differences = []
    for i in range(iterations):
        index_on = np.random.randint(0, len(h_ons))
        index_off = np.random.randint(0, len(h_offs))
        d = h_ons[index_on] - h_offs[index_off]
        actual_differences.append(d)
        
    actual_difference = np.nanmean(actual_differences, axis=0)
    
    all_hs = []
    all_hs.extend(h_ons)
    all_hs.extend(h_offs)
    
    differences = []
    for i in range(iterations):
        indices_on = np.random.randint(0, len(all_hs), neff)
        indices_off = np.random.randint(0, len(all_hs), neff)

        #fake_on = np.mean([all_hs[n] for n in indices_on], axis=0)
        #fake_off = np.mean([all_hs[n] for n in indices_off], axis=0)
        fake_on = np.array([all_hs[n] for n in indices_on])
        fake_off = np.array([all_hs[n] for n in indices_off])
        
        d = np.nanmean(fake_on - fake_off, axis=0)
        differences.append(d)
        
        #differences.append(fake_on-fake_off)
    differences = np.array(differences)
    
    pvals = np.ones_like(actual_difference)
    for r in range(0, differences.shape[1]):
        for c in range(0, differences.shape[2]):
            q = differences[:,r,c]
            q.sort()
            iq = np.argmin( np.abs(q-actual_difference[r,c]) )
            p = 1 - np.abs((iq - iterations/2.) / (iterations/2.))
            pvals[r,c] = p
            
    
    
    return actual_difference, pvals
    
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

def plot_individual_trajectories_from_dataset_format(dataset, keys, backgroundimage, ax=None, binsx=None, binsy=None):
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111)
        
    if backgroundimage is not None:
        backgroundimage = plt.imread(backgroundimage)
        binsx, binsy = get_bins_from_backgroundimage(backgroundimage)
        ax.imshow(backgroundimage, cmap=plt.get_cmap('gray'), extent=[binsx[0], binsx[-1], binsy[0], binsy[-1]])
        
    for key in keys:
        trajec = dataset.trajec(key)
        
        ax.plot(trajec.position_x, trajec.position_y)

