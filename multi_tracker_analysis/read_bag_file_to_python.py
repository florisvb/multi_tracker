import csv 
import numpy as np
from collections import namedtuple
import time
import pickle
import rosbag
import trajectory_analysis
import copy

class Trajectory(object):
    def __init__(self, objid):
        self.frames = []
        self.position = []
        self.velocity = []
        self.time = []
        self.length = 0
        self.objid = objid
        pass

def convert_bag_to_object(bagfile):
    bag = rosbag.Bag(bagfile)
    print 'Loaded: ', bagfile
    data = {}
    
    for topic, msg, t in bag.read_messages():
        for tracked_object in msg.tracked_objects:
            if tracked_object.objid in data.keys():
                trajec = data[tracked_object.objid]
            else:
                trajec = Trajectory(tracked_object.objid)
                data.setdefault(tracked_object.objid, trajec)
            
            trajec.position.append([tracked_object.position.x, tracked_object.position.y, tracked_object.position.z])
            trajec.velocity.append([tracked_object.velocity.x, tracked_object.velocity.y, tracked_object.velocity.z])
            trajec.time.append(tracked_object.header.stamp.secs + tracked_object.header.stamp.nsecs*1e-9)
            trajec.frames.append(msg.header.seq)
    print 'Done loading and converting data'    

    for key, trajec in data.items():
        trajec.position = np.array(trajec.position)
        trajec.velocity = np.array(trajec.velocity)
        trajec.time = np.array(trajec.time)
        trajec.frames = np.array(trajec.frames)
        trajec.length = len(trajec.frames)
    
    trajectory_analysis.calc_localtime(data)
    return data
    
def combine_datasets(datasets):
    primary_dataset = datasets[0]
    for d, dataset in enumerate(datasets[1:]):
        for key, trajec in dataset.items():
            new_key = key + '_' + str(d)
            trajec.objid = new_key
            primary_dataset.setdefault(new_key, trajec)
    return primary_dataset
    
def load_bag_as_dataset(bagfile):
    picklefile = bagfile + '.pickle'
    try:
        f = open(picklefile, 'w')
        pickle.load(f)
    except:
        data = convert_bag_to_object(bagfile)
        save_dataset(data, picklefile)
    
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
     

