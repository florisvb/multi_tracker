import read_hdf5_file_to_pandas as rhfp
import data_slicing
import numpy as np

def get_candidate_keys(pd, key, framerange=[2,10]):
    dataset = rhfp.Dataset(pd)
    last_frame = dataset.trajec(key).frames[-1]
    candidate_keys = data_slicing.get_keys_in_framerange(pd, [last_frame-framerange[0], last_frame+framerange[1]]).tolist()
    candidate_keys.remove(key)
    return candidate_keys
    
def check_distance_on_candidate_keys(pd, key, candidate_keys, max_distance=10):
    frame = np.max(pd[pd.objid==key].frames)
    pd_subset = data_slicing.get_data_in_framerange(pd, [frame-50, frame+50])
    dataset = rhfp.Dataset(pd_subset)
    key_position = np.array([dataset.trajec(key).position_x[-1], dataset.trajec(key).position_y[-1]])
    errors = []
    for candidate in candidate_keys:
        if candidate not in pd_subset.objid.values:
            e = np.inf
        # don't accept candidates that fully contain all the frames of the key
        #elif dataset.trajec(candidate).frames[0] < dataset.trajec(key).frames[0] and dataset.trajec(candidate).frames[-1] > dataset.trajec(key).frames[-1]:
        #    e = np.inf
        else:
            candidates_frames = dataset.trajec(candidate).frames
            index = np.argmin( np.abs(candidates_frames-dataset.trajec(key).frames[-1]) )
            candidate_position = np.array([dataset.trajec(candidate).position_x[index], dataset.trajec(candidate).position_y[index]])
            e = np.linalg.norm(key_position-candidate_position)
        errors.append(e)
    errors = np.array(errors)
    indices_ok = np.where(errors<max_distance)
    return np.array(candidate_keys)[indices_ok].tolist()
    
def get_distance_checked_candidate_keys(pd, key, framerange=[2,10], max_distance=10):
    candidate_keys = get_candidate_keys(pd, key, framerange)
    distance_checked_candidate_keys = check_distance_on_candidate_keys(pd, key, candidate_keys, max_distance)
    return distance_checked_candidate_keys

def get_link_chain(pd, key, framerange=[2,10], max_distance=14):
    chain = [key]
    candidate_keys = [key]
    while len(candidate_keys) == 1: 
        candidate_keys = get_candidate_keys(pd, chain[-1], framerange)
        candidate_keys = [k for k in candidate_keys if k not in chain]
        candidate_keys = check_distance_on_candidate_keys(pd, chain[-1], candidate_keys, max_distance)
        if len(candidate_keys) == 1:
            chain.append(candidate_keys[0])
    return chain

def get_complete_chain(pd, framerange=[2,10], max_distance=14):
    chains = []
    keys = np.unique(pd.objid)
    keys.sort()
    while len(keys) != 0:
        chain = get_link_chain(pd, keys[0], framerange, max_distance)
        print chain
        chains.append(chain)
        keys = [k for k in keys if k not in chain]
        keys.sort()
        pd = data_slicing.get_pd_subset_from_keys(pd, keys)
    return chains
    
def interpolate_keys(pd, keys):
    dataset = rhfp.Dataset(pd)
    first_frame = dataset.trajec(keys[0]).frames[0]
    last_frame = dataset.trajec(keys[-1]).frames[-1]
    new_idx = np.arange(first_frame, last_frame+1)
    
    pd_subset = data_slicing.get_pd_subset_from_keys(pd, keys)
    
    # now remove duplicate frames
    pd_subset = pd_subset.drop_duplicates('frames')
    
    # now interpolate missing frames
    d = np.diff(pd_subset.frames.values)
    if np.max(d) > 1:
        print 'reindexing'
        pd_subset.reindex(new_idx).interpolate()
        d = np.diff(pd_subset.frames.values)
        print np.max(d), ' if this value == 1, everything worked'
        
        
        
