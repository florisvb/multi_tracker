import csv 
import numpy as np
from collections import namedtuple
import time
import pickle
import h5py
import trajectory_analysis
import copy

class Trajectory(object):
    def __init__(self):
        pass

def convert_hdf5_to_object(hdf5):
    data = {}
    for key, obj in hdf5.items():
        last_frame = obj.attrs.get('current_frame') - 1
        first_frame = 0
        trajec_obj = Trajectory()
        
        trajec_obj.frames = obj['header.frame_id'][first_frame:last_frame]
        trajec_obj.position = np.hstack( (obj['position.x'], obj['position.y'], obj['position.z']) )[first_frame:last_frame]
        trajec_obj.velocity = np.hstack( (obj['velocity.x'], obj['velocity.y'], obj['velocity.z']) )[first_frame:last_frame]
        trajec_obj.size = obj['size'][first_frame:last_frame]
        trajec_obj.covariance = obj['covariance'][first_frame:last_frame]
        trajec_obj.time = (obj['header.stamp.secs'] + obj['header.stamp.nsecs']*1e-9)[first_frame:last_frame]
        trajec_obj.measurement = np.hstack( (obj['measurement.x'], obj['measurement.y']) )[first_frame:last_frame]

        trajec_obj.length = len(trajec_obj.frames)
        t_str = time.strftime('%Y%m%d_%H%M', time.localtime(trajec_obj.time[first_frame]))
        objid = t_str + '_' + key
        trajec_obj.objid = objid
        data.setdefault(objid, trajec_obj)
    trajectory_analysis.calc_localtime(data)
    return data
    
def load_data_as_python_object_from_hdf5_file(filename):
    f = h5py.File(filename, 'r')
    data_obj  = convert_hdf5_to_object(f)
    return data_obj
    
def combine_datasets(datasets):
    primary_dataset = datasets[0]
    for d, dataset in enumerate(datasets[1:]):
        for key, trajec in dataset.items():
            new_key = key + '_' + str(d)
            trajec.objid = new_key
            primary_dataset.setdefault(new_key, trajec)
    return primary_dataset
    
def save_dataset(dataset, filename):
    f = open(filename, 'w')
    pickle.dump(dataset, f)
    f.close()
        
def print_position_jumps(data, jumpsize=20):
    for key, trajec in data.items():
        v = np.linalg.norm(np.diff(trajec.position, axis=0), axis=1)
        if np.max(v) > jumpsize:
            print key
            
def split_jumps(data, jumpsize=20, dryrun=True):
    for key, trajec in data.items():
        v = np.linalg.norm(np.diff(trajec.position, axis=0), axis=1)
        if np.max(v) > jumpsize:
            jumps = np.where(v > jumpsize)[0].tolist()
            jumps.insert(0,0)
            jumps.append(trajec.length)
            
            new_trajecs = []
            for j, jump in enumerate(jumps[0:-1]):
                indices = [jump+1, jumps[j+1]]
                if indices[1] - indices[0] > 1:
                    if trajec.frames[indices[0]] != 0:
                        trajec_obj = Trajectory()
                        for attribute in trajec.__dict__.keys():
                            if attribute == 'objid':
                                newid = copy.copy(trajec.objid) + '_' + str(jump)
                                trajec_obj.objid = newid 
                            elif attribute == 'length':
                                pass
                            else:
                                trajec_obj.__setattr__(attribute, copy.copy(trajec.__getattribute__(attribute)[indices[0]:indices[1]]))
                        trajec.length = len(trajec.frames)
                        new_trajecs.append(trajec_obj)
                    
            if not dryrun:
                del(data[key])
                for new_trajec in new_trajecs:
                    print new_trajec.objid
                    data.setdefault(new_trajec.objid, new_trajec)

    return data
     

