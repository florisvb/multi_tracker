import numpy as np
import matplotlib.pyplot as plt
import data_slicing
from scipy.interpolate import interp1d
import trajectory_analysis

class Trajectory(object):
    def __init__(self):
        pass
        
def generate_relative_time_to_key_dict(data):
    time_to_key_dict = {}
    for key, trajec in data.items():
        t = int(trajec.time_relative[0])
        if t in time_to_key_dict.keys():
            time_to_key_dict[t].append(key)
        else:
            time_to_key_dict.setdefault(t, [key])
    return time_to_key_dict

def get_keys_in_relative_time_range(data, timerange, time_to_key_dict=None):
    if time_to_key_dict is None:
        time_to_key_dict = generate_relative_time_to_key_dict(data)
    keys = []
    for t in range(int(timerange[0]), int(timerange[-1])):
        if t in time_to_key_dict.keys():
            keys.extend(time_to_key_dict[t])
    return np.unique(keys)
                
def find_link_candidates_for_trajectory(data, key, timerange=[0,5], distance_range=10, time_to_key_dict=None, claimed_keys=[]):
    trajec = data[key]
    candidate_continuing_keys = get_keys_in_relative_time_range(data, [trajec.time_relative[-1]+timerange[0], trajec.time_relative[-1]+timerange[-1]], time_to_key_dict=time_to_key_dict)
    
    distance_errors = []
    time_errors = []
    candidate_keys = []
    for c, candidate_key in enumerate(candidate_continuing_keys):
        if candidate_key in claimed_keys:
            continue
        else:
            distance_error = np.linalg.norm(data[candidate_key].position[0] - trajec.position[-1])
            distance_errors.append(distance_error)
            time_errors.append( data[candidate_key].time[0] - trajec.time[-1] )
            candidate_keys.append(candidate_key)
    distance_errors = np.array(distance_errors)
    time_errors = np.array(time_errors)
    candidate_keys = np.array(candidate_keys)
    
    indices_with_positive_time_error = np.where(time_errors >= 0)[0]
    indices_with_nearby_starting_points = np.where(distance_errors[indices_with_positive_time_error] < distance_range)

    if len(    distance_errors[indices_with_positive_time_error][indices_with_nearby_starting_points] ) > 0:
        best_index = np.argmin(time_errors[indices_with_positive_time_error][indices_with_nearby_starting_points])
        return candidate_keys[indices_with_positive_time_error][indices_with_nearby_starting_points][best_index]
    else:
        return None
        
    #best_candidates = candidate_continuing_keys[indices_with_positive_time_error][indices_with_nearby_starting_points]
    #return best_candidates
    
def find_link_chain(data, key, timerange=[0,10], distance_range=20, maxlinks=5, time_to_key_dict=None, claimed_keys=[]):
    if time_to_key_dict is None:
        time_to_key_dict = generate_relative_time_to_key_dict(data)
        
    keys = [key]
    nlinks = 0
    while nlinks < maxlinks:
        best_link = find_link_candidates_for_trajectory(data, keys[-1], timerange=timerange, distance_range=distance_range, time_to_key_dict=time_to_key_dict, claimed_keys=claimed_keys)
        if best_link is not None:
            keys.append(best_link)
            claimed_keys.append(best_link)
            nlinks += 1
        else:
            break
            
    return keys, claimed_keys
    
    
def get_complete_link_chains(data, timerange=[0,10], distance_range=20, maxlinks=1000):
    time_to_key_dict = generate_relative_time_to_key_dict(data)
    
    seed_keys = data_slicing.get_keys_in_relative_time_range(data,[-1000, 3600])
    keys_accounted_for = []
    chains = []
    claimed_keys = []
    for seed_key in seed_keys:
        if seed_key in keys_accounted_for:
            continue
        
        links, claimed_keys = find_link_chain(data, seed_key, timerange=timerange, distance_range=distance_range, maxlinks=maxlinks, time_to_key_dict=time_to_key_dict, claimed_keys=claimed_keys)
        chains.append(links)
        keys_accounted_for.extend(links)
        print len(links)
    
    return chains

def find_duplicates_in_chains(chains):
    duplicates = []
    
    for c, chain in enumerate(chains):
        for key in chain:
            if len(chains) > c+1:
                for chain2 in chains[c+1:]:
                    if key in chain2:
                        duplicates.append(key)
    
    return duplicates

def plot_trajec_and_candidate_links(data, key, candidates):
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    
    ax.plot(data[key].position[:,0], data[key].position[:,1], color='red')
    ax.plot(data[key].position[-1,0], data[key].position[-1,1], 'o', color='red')
    
    for key in candidates:
        ax.plot(data[key].position[:,0], data[key].position[:,1], color='blue')
        ax.plot(data[key].position[0,0], data[key].position[0,1], 'o', color='green')

def link_trajectories_into_one(data, keys):
    linked_trajec = Trajectory()

    # init    
    key = keys[0]
    linked_trajec.frames =     data[key].frames
    linked_trajec.position =   data[key].position
    linked_trajec.velocity =   data[key].velocity
    linked_trajec.time =       data[key].time
    linked_trajec.objid =      key
    linked_trajec.length =     data[key].length

    for key in keys[1:]:
        trajec = data[key]
        n_frames_elapsed = trajec.frames[0] - linked_trajec.frames[-1]
        
        if n_frames_elapsed > 1:
            frames_to_interpolate = np.arange(linked_trajec.frames[-1]+1, linked_trajec.frames[-1]+n_frames_elapsed)
            # interpolate
            interp_position = interp1d([linked_trajec.frames[-1], trajec.frames[0]], [linked_trajec.position[-1], trajec.position[0]], axis=0)
            interp_velocity = interp1d([linked_trajec.frames[-1], trajec.frames[0]], [linked_trajec.velocity[-1], trajec.velocity[0]], axis=0)
            interp_time = interp1d([linked_trajec.frames[-1], trajec.frames[0]], [linked_trajec.time[-1], trajec.time[0]], axis=0)
            
            interpolated_position = interp_position(frames_to_interpolate)
            linked_trajec.position = np.vstack( (linked_trajec.position, interpolated_position) )
            
            interpolated_velocity = interp_velocity(frames_to_interpolate)
            linked_trajec.velocity = np.vstack( (linked_trajec.velocity, interpolated_velocity) )
            
            interpolated_time = interp_time(frames_to_interpolate)
            linked_trajec.time = np.hstack( (linked_trajec.time, interpolated_time) )

            linked_trajec.frames = np.hstack( (linked_trajec.frames, frames_to_interpolate) )
        
        # append
        linked_trajec.frames = np.hstack( (linked_trajec.frames, trajec.frames) )
        linked_trajec.position = np.vstack( (linked_trajec.position, trajec.position) )
        linked_trajec.velocity = np.vstack( (linked_trajec.velocity, trajec.velocity) )
        linked_trajec.time = np.hstack( (linked_trajec.time, trajec.time) )
        linked_trajec.objid += '_'
        linked_trajec.objid += key.split('_')[1]
        linked_trajec.length = len(linked_trajec.frames)
        
    return linked_trajec
        
def get_linked_dataset(config, data, key_chains):
    linked_data = {}
    for key_chain in key_chains:
        linked_trajec = link_trajectories_into_one(data, key_chain)
        linked_data.setdefault(linked_trajec.objid, linked_trajec)
    trajectory_analysis.calc_localtime(linked_data)
    trajectory_analysis.calc_relative_time(linked_data, config.timestamp_reference)
    return linked_data
    
def link_data(config, data, timerange=[0,30], distance_range=30, maxlinks=100000):
    chains = get_complete_link_chains(data, timerange=timerange, distance_range=distance_range, maxlinks=maxlinks)
    linked_data = get_linked_dataset(config, data, chains)
    return linked_data
    
def link_datasets(configs, datasets, timerange=[0,10], distance_range=30, maxlinks=100000):
    linked_datasets = {}
    for name in datasets.keys():
        data = datasets[name]
        linked_data = link_data(configs[name], data, timerange=timerange, distance_range=distance_range, maxlinks=maxlinks)
        linked_datasets.setdefault(name, linked_data)
    return linked_datasets
    
'''


############
    
def collect_start_and_end_points(data):
    keys = []
    start_points = [] 
    end_points = []
    for key, trajec in data.items():
        keys.append(key)
        start_points.append(trajec.time[0])
        end_points.append(trajec.time[-1])
    return keys, np.array(start_points), np.array(end_points) 
    
def find_candidate_links(data, time_range=0.05, position_range=4):
    keys, start_points, end_points = collect_start_and_end_points(data)

    diff_matrix = np.ones([len(start_points), len(end_points)])
    for i in range(len(end_points)):
        diff_matrix[i,:] = start_points - end_points[i] # note: end points correspond to starting trajecs, start points to continuing trajecs
    
    starting_trajec_indices, continuing_trajec_indices = np.where( (diff_matrix < time_range)*(diff_matrix > 0) )
    
    link_dict = {}
    
    for i in range(len(starting_trajec_indices)):
        starting_trajec = data[ keys[starting_trajec_indices[i]] ]
        continuing_trajec = data[ keys[continuing_trajec_indices[i]] ]
        
        if starting_trajec.objid == continuing_trajec.objid:
            continue
        
        position_difference = np.linalg.norm( starting_trajec.position[-1] - continuing_trajec.position[0] )
        
        print keys[starting_trajec_indices[i]], keys[continuing_trajec_indices[i]]
        print continuing_trajec.time[0] - starting_trajec.time[-1]
        print position_difference
        print

        if position_difference < position_range:
            if starting_trajec.objid not in link_dict.keys():
                link_dict.setdefault(starting_trajec.objid, [])
            link_dict[starting_trajec.objid].append(continuing_trajec.objid)
    
    return link_dict
    
def plot_candidate_link_chain(data, link_dict, seedkey):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    
    chaining = True
    keys = [seedkey]
    while chaining: 
        for key in keys:   
            ax.plot( data[key].position[:,0], data[key].position[:,1] )
        chaining = False
        
        new_keys = []
        for key in keys:
            if key in link_dict.keys():
                new_keys.extend( link_dict[key] )
                chaining = True
        keys = new_keys

def get_link_chain(link_dict, seedkey):
    keys = [seedkey]
    chaining = True
    key = seedkey
    while chaining: 
        chaining = False
        new_key = None
        if key in link_dict.keys():
            new_keys = link_dict[key]
            if len(new_keys) > 1:
                print 'too many choices, seed: ', key, ' children: ', new_keys
            else:
                if new_keys[0] not in keys:
                    key = new_keys[0]
                    keys.append(key)
                    chaining = True
                
    return keys
    
def get_all_link_chains(link_dict, data):
    keys_accounted_for = []
    keys, start_points, end_points = collect_start_and_end_points(data)
    keys_order = np.argsort(start_points)
    keys_in_order = np.array(keys)[keys_order]
    key_chains = []
    
    for key in keys_in_order:
        if key in keys_accounted_for:
            continue
        chain = get_link_chain(link_dict, key)
        key_chains.append(chain)
        keys_accounted_for.extend(chain)
    print
    print 'CHECK: '
    print 'total keys: ', len(keys_in_order)
    print 'keys accounted for: ', len(keys_accounted_for)
    return key_chains         
        
def link_trajectories_into_one(data, keys):
    linked_trajec = Trajectory()

    # init    
    key = keys[0]
    linked_trajec.frames =     data[key].frames
    linked_trajec.position =   data[key].position
    linked_trajec.velocity =   data[key].velocity
    linked_trajec.time =       data[key].time
    linked_trajec.objid =      key
    linked_trajec.length =     data[key].length

    for key in keys[1:]:
        trajec = data[key]
        n_frames_elapsed = trajec.frames[0] - linked_trajec.frames[-1]
        
        if n_frames_elapsed > 1:
            frames_to_interpolate = np.arange(linked_trajec.frames[-1]+1, linked_trajec.frames[-1]+n_frames_elapsed)
            # interpolate
            interp_position = interp1d([linked_trajec.frames[-1], trajec.frames[0]], [linked_trajec.position[-1], trajec.position[0]], axis=0)
            interp_velocity = interp1d([linked_trajec.frames[-1], trajec.frames[0]], [linked_trajec.velocity[-1], trajec.velocity[0]], axis=0)
            interp_time = interp1d([linked_trajec.frames[-1], trajec.frames[0]], [linked_trajec.time[-1], trajec.time[0]], axis=0)
            
            interpolated_position = interp_position(frames_to_interpolate)
            linked_trajec.position = np.vstack( (linked_trajec.position, interpolated_position) )
            
            interpolated_velocity = interp_velocity(frames_to_interpolate)
            linked_trajec.velocity = np.vstack( (linked_trajec.velocity, interpolated_velocity) )
            
            interpolated_time = interp_time(frames_to_interpolate)
            linked_trajec.time = np.hstack( (linked_trajec.time, interpolated_time) )

            linked_trajec.frames = np.hstack( (linked_trajec.frames, frames_to_interpolate) )
        
        # append
        linked_trajec.frames = np.hstack( (linked_trajec.frames, trajec.frames) )
        linked_trajec.position = np.vstack( (linked_trajec.position, trajec.position) )
        linked_trajec.velocity = np.vstack( (linked_trajec.velocity, trajec.velocity) )
        linked_trajec.time = np.hstack( (linked_trajec.time, trajec.time) )
        linked_trajec.objid += '_'
        linked_trajec.objid += key.split('_')[1]
        linked_trajec.length = len(linked_trajec.frames)
        
    return linked_trajec
        
def get_linked_dataset(data, key_chains):
    linked_data = {}
    for key_chain in key_chains:
        linked_trajec = link_trajectories_into_one(data, key_chain)
        linked_data.setdefault(linked_trajec.objid, linked_trajec)
    return linked_data
    
def get_linked_dataset_for_position_time(data, time_range=0.05, position_range=6):
    link_dict = find_candidate_links(data, time_range, position_range)
    key_chains = get_all_link_chains(link_dict, data)
    linked_data = get_linked_dataset(data, key_chains)
    print
    print len(data.keys())
    print len(linked_data.keys())
    return linked_data
        
def iterate_linking(data, iterations=3, time_range=0.05, position_range=6):
    for iteration in range(iterations):
        link_dict = find_candidate_links(data, time_range, position_range)
        key_chains = get_all_link_chains(link_dict, data)
        data = get_linked_dataset(data, key_chains)
    return data
    
def multistage_iterated_linking(data, position_range=10, time_ranges = [0.1, 0.2, 0.5, 1, 2, 5, 10]):
    linked_data = data
    for time_range in time_ranges:
        linked_data = iterate_linking(linked_data, iterations=3, time_range=time_range, position_range=position_range)
        print 'N keys: ', len(linked_data.keys())
    return linked_data

def plot_linked_keys(data):
    
    for key, trajec in data.items():
        fig = plt.figure()
        ax = fig.add_subplot(111)
        
        ax.plot(trajec.position[:,0], trajec.position[:,1])
        
        
def plot_potential_links(key, link_dict, data):
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    
    starting_trajec = data[key]
    first_frame = np.max([0, starting_trajec.length-300])
    ax.plot(starting_trajec.position[first_frame:starting_trajec.length,0], starting_trajec.position[first_frame:starting_trajec.length,1], color='green', linewidth=3)
    
    for k in link_dict[key]:
        trajec = data[k]
        first_frame = 0
        last_frame = np.min([300, trajec.length])
        ax.plot(trajec.position[first_frame:last_frame,0], trajec.position[first_frame:last_frame,1])

        position_difference = np.linalg.norm( starting_trajec.position[-1] - trajec.position[0] )
        print k
        print position_difference
        print trajec.time[0] - starting_trajec.time[-1]
        
def cull_tiny_trajecs(data, cull_length=5):
    for key, trajec in data.items():
        if trajec.length < cull_length:
            del(data[key])
        
        
        
        
        
'''




