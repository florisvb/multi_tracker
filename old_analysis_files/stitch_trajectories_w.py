import multi_tracker_analysis.data_slicing_w as data_slicing
import h5py

def get_candidate_link_keys(data, key, distance, framedistance):
    point = [data.trajec(key).position[-1,0], data.trajec(key).position[-1,1]]
    candidates = data_slicing.get_keys_close_to_point_and_frame(data, point, distance, data.trajec(key).frames[-1], framedistance)
    return candidates

def get_link_chain(data, key, distance, framedistance):
    chain = []
    candidates = [key]
    while len(candidates) > 0:
        chain.append(candidates[0])
        print chain
        candidates = get_candidate_link_keys(data, candidates[0], distance, framedistance)
    return chain
    
def get_empty_h5_like(hdf5data, filename):
    hdf5 = h5py.File(filename, 'w')
    hdf5.attrs.create("info", info)
    hdf5.create_dataset('data', (self.chunk_size, 1), maxshape=(None,1), dtype=hdf5data.dtype)
    return hdf5

# for interpolating trajectories
#for datatype in data.hdf5data.dtype.fields.keys():
#    np.interp(np.arange(478,490), [478,490], [data.trajec(2).hdf5trajec[-1]['position.x'], data.trajec(9).hdf5trajec[0]['position.x']])
