import h5py
import os
import numpy as np
import time

class Trajectory:
    def __init__(self, hdf5data, objid, start_time=0):
        self.objid_indices = np.where(hdf5data['objid']==objid)
        self.hdf5trajec = hdf5data[:][self.objid_indices]
        self.start_time = start_time
        self.position = self.__position__()
        self.velocity = self.__velocity__()
        self.length = len(self.position)
        self.time_epoch = self.__time_epoch__()
        self.time = self.__time__()
        self.time_local = self.__time_local__()
        self.frames = self.__frames__()
        self.date = self.__date__()
    def __position__(self):
        position = np.array( (self.hdf5trajec['position.x'], self.hdf5trajec['position.y'], self.hdf5trajec['position.z'])).T
        return position
    def __velocity__(self):
        velocity = np.array( (self.hdf5trajec['velocity.x'], self.hdf5trajec['velocity.y'], self.hdf5trajec['velocity.z'])).T
        return velocity
    def __time_epoch__(self):
        t = self.hdf5trajec['header.stamp.secs'] + self.hdf5trajec['header.stamp.nsecs']*1e-9 
        return t
    def __time__(self):
        return self.time_epoch - self.time_epoch[0]
    def __time_local__(self):
        t = self.time_epoch[0]
        lt = time.localtime(t).tm_hour + time.localtime(t).tm_min/60. + time.localtime(t).tm_sec/3600.
        return self.time/3600. + lt
    def __frames__(self):
        f = self.hdf5trajec['header.frame_id']
        return f
    def __date__(self):
        lt = time.localtime(self.time_epoch)
        return str(lt.tm_year) + str(lt.tm_mon) + str(lt.tm_day) 
        
class Dataset:
    def __init__(self):
        self.keys = []
        self.__processed_trajecs__ = {}
    
    def trajec(self, key):
        if key in self.__processed_trajecs__.keys():
            return self.__processed_trajecs__[key]
        else:
            hdf5trajec = self.hdf5data[key]
            self.__processed_trajecs__.setdefault(key, Trajectory(self.hdf5data, key, start_time=self.start_time))
            return self.__processed_trajecs__[key]
    
    def add_hdf5_dataset(self, hdf5_filename):
        hdf5data = h5py.File(hdf5_filename, 'r')['data']
        self.hdf5data = np.delete(hdf5data,np.where(hdf5data['header.frame_id']==0))
        self.keys = np.unique(self.hdf5data['objid'])
        t = self.hdf5data[0]['header.stamp.secs'] + self.hdf5data[0]['header.stamp.nsecs']*1e-9 
        self.start_time = t
        
def load_data_as_python_object_from_hdf5_file(datafile):
    dataset = Dataset()
    dataset.add_hdf5_dataset(datafile)
    return dataset
        
        
        
        
        
        
        
        
        
        
