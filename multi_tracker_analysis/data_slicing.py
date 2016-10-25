import numpy as np

def get_keys_in_framerange(pd, framerange):
    return np.unique(pd.ix[framerange[0]:framerange[-1]].objid)

def get_frames_for_key(pd, key):
    return pd[pd.objid==key].frames.values
    
def get_data_in_framerange(pd, framerange):
    # pd_subset
    return pd.ix[framerange[0]:framerange[-1]]

def get_data_in_epoch_timerange(pd, timerange):
    # pd_subset
    return pd[(pd.time_epoch>timerange[0]) & (pd.time_epoch<timerange[1])]
    
def get_nframes_per_key(pd):
    first_key = np.min(pd.objid)
    last_key = np.max(pd.objid)
    bins = np.arange(first_key, last_key+2, dtype=float)
    bins -= 0.5
    h, b = np.histogram(pd.objid, bins)
    keys = np.arange(first_key, last_key+1, dtype=int)
    return keys, h
    
def get_nkeys_per_frame(pd):
    first_key = np.min(pd.frames)
    last_key = np.max(pd.frames)
    bins = np.arange(first_key, last_key, dtype=float)
    bins -= 0.5
    h, b = np.histogram(pd.frames, bins)
    # can use pd.frames.groupby(pd.frames).agg('count')
    return h
    
def calc_frames_with_object_in_circular_region(pd, center, radius, region_name='region'):
    '''
    center  - list (x,y) units should match units of position_x and position_y
    '''
    x = pd.position_x
    y = pd.position_y
    r0 = (center[0]-x)**2 + (center[1]-y)**2
    indices = np.where( r0<= radius**2 )
    pd[region_name] = np.zeros_like(pd.position_x)
    pd[region_name].iloc[indices] = 1
    return pd

def calc_frames_with_object_NOT_in_circular_region(pd, center, radius, region_name='region'):
    '''
    center  - list (x,y) units should match units of position_x and position_y
    '''
    x = pd.position_x
    y = pd.position_y
    r0 = (center[0]-x)**2 + (center[1]-y)**2
    indices = np.where( r0> radius**2 )
    pd[region_name] = np.zeros_like(pd.position_x)
    pd[region_name].iloc[indices] = 1
    return pd
    
def remove_objects_that_enter_area_outside_circular_region(pd, center, radius, region_name='outofbounds'):
    pd = calc_frames_with_object_NOT_in_circular_region(pd, center, radius, region_name=region_name)
    outofbounds = np.unique(pd[pd[region_name]==1].objid.values)
    keys_ok = [key for key in pd.objid if key not in outofbounds]
    indices_where_object_acceptable = pd.objid.isin(keys_ok)
    culled_pd = pd[indices_where_object_acceptable]
    return culled_pd
        
def calc_frames_with_object_in_rectangular_region(pd, x_range, y_range, z_range=None, region_name='region'):
    '''
    center  - list (x,y) units should match units of position_x and position_y
    '''
    if z_range is None:
        x = pd.position_x
        y = pd.position_y
        indices = np.where( (x>x_range[0]) & (x<x_range[-1]) & (y>y_range[0]) & (y<y_range[-1]) )
    else:
        x = pd.position_x
        y = pd.position_y
        z = pd.position_z
        indices = np.where( (x>x_range[0]) & (x<x_range[-1]) & (y>y_range[0]) & (y<y_range[-1]) & (z>z_range[0]) & (z<z_range[-1]) )
    
    pd[region_name] = np.zeros_like(pd.position_x)
    pd[region_name].iloc[indices] = 1
    
    return pd
    
def get_pd_subset_from_keys(pd, keys):
    pd_subset = pd.query('objid in @keys')
    return pd_subset
    

    
    
    
    
    
    
    
    
    
    
