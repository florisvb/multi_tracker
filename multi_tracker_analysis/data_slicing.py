import numpy as np

def get_dataset_from_keys(data, keys):
    new_data = {}
    for key in keys:
        new_data.setdefault(key, data[key])
    return new_data

def get_keys_with_attributes(data, attributes, values, keys=None):
    if keys is None:
        keys = data.keys()
        
    if type(attributes) is not list:
        attributes = [attributes]
        values = [values]
    
    keys_with_attributes = []
    for key in keys:
        trajec = data[key]
        key_ok = True
        for a, attribute in enumerate(attributes):
            if trajec.__getattribute__(attribute) == values[a]:
                pass
            else:
                key_ok = False
                break
        if key_ok:
            keys_with_attributes.append(key)
        
    return keys_with_attributes
        
def get_keys_of_length_greater_than(data, length, keys=None):
    if keys is None:
        keys = data.keys()
        
    keys_greater_than = []
    for key in keys:
        trajec = data[key]
        if trajec.length > length:
            keys_greater_than.append(key)
    return keys_greater_than

def get_keys_in_local_time_range(data, local_time_range, keys=None):
    if keys is None:
        keys = data.keys()
        
    keys_in_time_range = []
    for key in keys:
        trajec = data[key]
        if trajec.time_local[0] > local_time_range[0] and trajec.time_local[0] < local_time_range[-1]:
            keys_in_time_range.append(key)
    return keys_in_time_range
    
def get_keys_in_continuous_local_time_range(data, local_time_range, keys=None):
    if keys is None:
        keys = data.keys()
        
    keys_in_time_range = []
    for key in keys:
        trajec = data[key]
        
        # if trajectory entirely contained in local range
        if trajec.time_continuous_local[0] > local_time_range[0] and trajec.time_continuous_local[-1] < local_time_range[-1]:
            keys_in_time_range.append(key)
            
        # if trajectory contains first point in range
        elif trajec.time_continuous_local[-1] > local_time_range[0] and trajec.time_continuous_local[-1] < local_time_range[-1]:
            keys_in_time_range.append(key)
        
        # if trajectory contains last point in range
        elif trajec.time_continuous_local[0] > local_time_range[0] and trajec.time_continuous_local[0] < local_time_range[-1]:
            keys_in_time_range.append(key) 
            
        # if trajectory contains entire range
        elif trajec.time_continuous_local[0] < local_time_range[0] and trajec.time_continuous_local[-1] > local_time_range[-1]:
            keys_in_time_range.append(key) 
            
    return keys_in_time_range
    
def get_keys_in_speed_range(data, speed_range, keys=None):
    if keys is None:
        keys = data.keys()
    
    keys_in_speed_range = []
    for key in keys:
        trajec = data[key]
        if np.min(trajec.speed) > np.min(speed_range) and np.max(trajec.speed) < np.max(speed_range):
            keys_in_speed_range.append(key)
    
    return keys_in_speed_range
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
