import numpy as np

def get_keys_in_framerange(pd, framerange):
    return np.unique(pd.ix[framerange[0]:framerange[-1]].objid)

def get_frames_for_key(pd, key):
    return pd[pd.objid==22].frames.values
    
def get_data_in_framerange(pd, framerange):
    # pd_subset
    return pd.ix[framerange[0]:framerange[-1]]
    
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
    bins = np.arange(first_key, last_key+2, dtype=float)
    bins -= 0.5
    h, b = np.histogram(pd.frames, bins)
    return h
