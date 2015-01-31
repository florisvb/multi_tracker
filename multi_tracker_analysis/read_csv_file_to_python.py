import csv 
import numpy as np
from collections import namedtuple

OBJECT_IDENTIFIER = 2

def read_csv_file_to_dictionary(filename):
    data = {}
    with open(filename, 'rb') as csvfile:
        datareader = csv.reader(csvfile, delimiter=',', quotechar='|')
        r = -1
        for row in datareader:
            r += 1
            if r == 0:
                continue
            if r == 1:
                labels = row
                continue
            if r == 2:
                formats = row
                continue
            
            objid = row[OBJECT_IDENTIFIER]
            
            if objid not in data.keys():
                data.setdefault(objid, {})
                trajectory = data[objid]
                for label in labels:
                    trajectory.setdefault(label, [])
            
            trajectory = data[objid]
            for l, label in enumerate(labels):
                # convert to appropriate format
                if formats[l] == 'int':
                    value = int(row[l])
                elif formats[l] == 'float':
                    value = float(row[l])
                else:
                    value = row[l]
                
                # append data to appropriate label
                trajectory[label].append(value)
                
    return data     
            
def convert_data_dict_to_object(data_dict):
    data = {}
    for key, trajec_dict in data_dict.items():
        trajec_obj = namedtuple('Trajectory', [])
        trajec_obj.objid = key
        trajec_obj.position = np.array([trajec_dict['position.x'], trajec_dict['position.y'], trajec_dict['position.z']]).T
        trajec_obj.velocity = np.array([trajec_dict['velocity.x'], trajec_dict['velocity.y'], trajec_dict['velocity.z']]).T
        trajec_obj.measured_position = np.array([trajec_dict['measurement.x'], trajec_dict['measurement.y']]).T
        trajec_obj.time = np.array(trajec_dict['header.stamp.secs'], dtype=float) + np.array(trajec_dict['header.stamp.nsecs'], dtype=float)*1e-9
        trajec_obj.length = len(trajec_obj.position)
        data.setdefault(key, trajec_obj)
    return data
    
def load_data_as_python_object_from_csv_file(filename)  :
    data_dict = read_csv_file_to_dictionary(filename)
    data_obj  = convert_data_dict_to_object(data_dict)
    return data_obj
    
