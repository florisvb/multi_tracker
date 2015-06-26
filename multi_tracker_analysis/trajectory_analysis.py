import time
import numpy as np
import matplotlib.pyplot as plt

def get_localtime(t):
    lt = time.localtime(np.floor(t))
    remainder = t - np.floor(t)
    return lt.tm_hour + lt.tm_min/60. + lt.tm_sec/3600. + remainder/3600.
        
def calc_localtime(data):
    for key, trajec in data.items():
        trajec.time_local = np.zeros_like(trajec.time)
        for i, t in enumerate(trajec.time):
            localtime = get_localtime(t)
            trajec.time_local[i] = localtime

def calc_continuous_localtime(data, rolloverthreshold=12):
    for key, trajec in data.items():
        trajec.time_continuous_local = np.zeros_like(trajec.time)
        for i, t in enumerate(trajec.time):
            localtime = trajec.time_local[i]
            if localtime < rolloverthreshold:
                t = localtime + 24
            else:
                t = localtime
            trajec.time_continuous_local[i] = t

def calc_relative_time(data, timestamp_relative):
    for key, trajec in data.items():
        trajec.time_relative = trajec.time - timestamp_relative
    
def calc_speed(data):
    for key, trajec in data.items():
        trajec.speed = np.linalg.norm(trajec.velocity, axis=1)
        
def get_length_stats(data):
    length = []
    for key, trajec in data.items():
        length.append(trajec.length)
    print 'mean: ', np.mean(length), 'std: ', np.std(length)
    print 'min: ', np.min(length), 'max: ', np.max(length)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.hist(length, bins=50)
    
    
                        
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
