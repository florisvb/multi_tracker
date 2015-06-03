import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

class Trajectory(object):
    def __init__(self):
        pass

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
        diff_matrix[i,:] = np.abs( start_points - end_points[i] )
    
    starting_trajec_indices, continuing_trajec_indices = np.where(diff_matrix < time_range)
    
    link_dict = {}
    
    for i in range(len(starting_trajec_indices)):
        starting_trajec = data[ keys[starting_trajec_indices[i]] ]
        continuing_trajec = data[ keys[continuing_trajec_indices[i]] ]
        
        if starting_trajec.objid == continuing_trajec.objid:
            continue
        
        position_difference = np.linalg.norm( starting_trajec.position[-1] - continuing_trajec.position[0] )
        print position_difference
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
        
def iterate_linking(data, iterations=3, time_range=0.05, position_range=6):
    for iteration in range(iterations):
        link_dict = find_candidate_links(data, time_range, position_range)
        key_chains = get_all_link_chains(link_dict, data)
        data = get_linked_dataset(data, key_chains)
    return data

def plot_linked_keys(data):
    
    for key, trajec in data.items():
        fig = plt.figure()
        ax = fig.add_subplot(111)
        
        ax.plot(trajec.position[:,0], trajec.position[:,1])
        
        
        
        
        
        
        
        
        
        
        




