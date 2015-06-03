import time
import numpy as np

def get_localtime(t):
    lt = time.localtime(t)
    return lt.tm_hour + lt.tm_min/60. + lt.tm_sec/3600.
        
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
            
    
def calc_speed(data):
    for key, trajec in data.items():
        trajec.speed = np.linalg.norm(trajec.velocity, axis=1)
