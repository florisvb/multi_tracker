import csv 
import numpy as np
from collections import namedtuple
import time
import pickle

import trajectory_analysis

class Trajectory(object):
    def __init__(self):
        pass

def convert_hdf5_to_object(hdf5):
    data = {}
    for key, obj in hdf5.items():
        last_frame = obj.attrs.get('current_frame') + 1
        trajec_obj = Trajectory()
        trajec_obj.frames =     np.array( obj['header.seq'][0:last_frame] )
        trajec_obj.position =   np.array( obj['position'][0:last_frame] )
        trajec_obj.velocity =   np.array( obj['velocity'][0:last_frame] )
        trajec_obj.size =       np.array( obj['size'][0:last_frame] )
        trajec_obj.measured_position = np.array( obj['measurement'][0:last_frame] )
        trajec_obj.time =       np.array( obj['header.stamp.secs'] ).astype(float) + np.array( obj['header.stamp.nsecs'] ).astype(float)*1e-9
        trajec_obj.length = last_frame
        t_str = time.strftime('%Y%m%d', time.localtime(trajec_obj.time[0]))
        objid = t_str + '_' + key
        trajec_obj.objid = objid
        data.setdefault(objid, trajec_obj)
    trajectory_analysis.calc_localtime(data)
    return data
    
def combine_datasets(datasets):
    primary_dataset = datasets[0]
    for dataset in datasets[1:]:
        for key, trajec in dataset.items():
            primary_dataset.setdefault(key, trajec)
    return primary_dataset
    
def save_dataset(dataset, filename):
    f = open(filename, 'w')
    pickle.dump(dataset, f)
    f.close()
        
        
    
    
    
    
    
    
    
    
    
