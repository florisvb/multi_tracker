import numpy as np
import h5py
import copy
import pandas
import os
import imp
import pickle

def get_filenames(path, contains):
    cmd = 'ls ' + path
    ls = os.popen(cmd).read()
    all_filelist = ls.split('\n')
    try:
        all_filelist.remove('')
    except:
        pass
    filelist = []
    for i, filename in enumerate(all_filelist):
        if contains in filename:
            filelist.append( os.path.join(path, filename) )
    return filelist
    
def get_filename(path, contains):
    cmd = 'ls ' + path
    ls = os.popen(cmd).read()
    all_filelist = ls.split('\n')
    try:
        all_filelist.remove('')
    except:
        pass
    filelist = []
    for i, filename in enumerate(all_filelist):
        if contains in filename:
            return os.path.join(path, filename)
            
def load_bag_as_hdf5(bag, skip_messages=[]):
    output_fname = bag.split('.')[0] + '.hdf5'
    print output_fname
    if not os.path.exists(output_fname):
        mta.bag2hdf5.bag2hdf5(   bag,
                                 output_fname,
                                 max_strlen=200,
                                 skip_messages=skip_messages)    
    metadata = h5py.File(output_fname, 'r')
    return metadata

class Trajectory(object):
    def __init__(self, pd, objid):
        self.pd = pd[pd['objid']==objid]
        
        for column in self.pd.columns:
            self.__setattr__(column, self.pd[column].values)
        
class Dataset(object):
    def __init__(self, pd):
        self.pd = pd
        self.keys = []
        self.__processed_trajecs__ = {}
    
    def trajec(self, key):
        return Trajectory(self.pd, key)
    
    def framestamp_to_timestamp(self, frame):
        t = self.pd.ix[frame]['time_epoch']
        try:
            return t.iloc[0]
        except:
            return t
            
    def timestamp_to_framestamp(self, t):
        d = self.pd['time_epoch'] - t
        indices = np.where(d<0)
        d.iloc[indices] = np.inf
        return np.argmin(d)
        
    def load_keys(self, keys=None):
        if keys is None:
            self.keys = np.unique(self.pd.objid).tolist()
        else:
            self.keys = keys
            
def load_data_as_pandas_dataframe_from_hdf5_file(filename, attributes=None):
    if '.pickle' in filename:
        pd = pandas.read_pickle(filename)
        return pd
    data = h5py.File(filename, 'r', swmr=True)['data']
    if attributes is None:
        attributes = {   'objid'                : 'objid',
                         'time_epoch_secs'      : 'header.stamp.secs',
                         'time_epoch_nsecs'     : 'header.stamp.nsecs',
                         'position_x'           : 'position.x',
                         'position_y'           : 'position.y',
                         'measurement_x'        : 'measurement.x',
                         'measurement_y'        : 'measurement.y',
                         'velocity_x'           : 'velocity.x',
                         'velocity_y'           : 'velocity.y',
                         'angle'                : 'angle',
                         'frames'               : 'header.frame_id',
                         'area'                 : 'size',
                         }
    index = data['header.frame_id'].flat
    d = {}
    for attribute, name in attributes.items():
        d.setdefault(attribute, data[name].flat)
    pd = pandas.DataFrame(d, index=index)
    pd = pd.drop(pd.index==[0]) # delete 0 frames (frames with no data)
    pd = calc_additional_columns(pd)
    # pd_subset = pd[pd.objid==key]
    return pd
    
def load_and_preprocess_data(hdf5_filename):
    '''
    requires that a configuration file be found in the same directory as the hdf5 file, with the same prefix
    
    returns: pandas dataframe, processed according to configuration file, and the configuration file instance
    '''
    pd = load_data_as_pandas_dataframe_from_hdf5_file(hdf5_filename, attributes=None)
    
    hdf5_basename = os.path.basename(hdf5_filename)
    directory = os.path.dirname(hdf5_filename)
    
    identifiercode = hdf5_basename.split('_trackedobjects')[0]
    config_filename = 'config_' + identifiercode + '.py'
    config_filename = get_filename(directory, config_filename)

    if config_filename is not None:
        Config = imp.load_source('Config', config_filename)
        config = Config.Config(directory, identifiercode)
        if config.__dict__.has_key('preprocess_data_function'):
            pd = config.__getattribute__('preprocess_data_function')(pd)
    else:
        config = None
        
    return pd, config
    
def delete_cut_join_trajectories_according_to_instructions(pd, instructions_filename):
    f = open(instructions_filename)
    instructions = pickle.load(f)
    f.close()
    
    for instruction in instructions:
        if instruction['action'] == 'delete':
            pd = pd[pd.objid!=instruction['objid']]
        elif instruction['action'] == 'cut':
            mask = (pd['objid']==instruction['objid']) & (pd['frames']>instruction['cut_frame_global'])
            pd.loc[mask,'objid'] = instruction['new_objid']
        elif instruction['action'] == 'join':
            for key in instruction['objids']:
                mask = pd['objid']==key
                pd.loc[mask,'objid'] = instruction['objids'][0]
    return pd
    
def calc_additional_columns(pd):
    pd['time_epoch'] = pd['time_epoch_secs'] + pd['time_epoch_nsecs']*1e-9
    pd['speed'] = np.linalg.norm( [pd['velocity_x'], pd['velocity_y']], axis=0 )
    return pd
    
def framestamp_to_timestamp(pd, frame):
    return pd.ix[frame]['time_epoch'].iloc[0]
        
def timestamp_to_framestamp(pd, t):
    pd_subset = pd[pd['time_epoch_secs']==np.floor(t)]
    return np.argmin(pd_subset['time_epoch'] - t)

def pixels_to_units(pd, pixels_per_unit, center=[0,0]):
    attributes = ['speed',
                  'position_x',
                  'position_y',
                  'velocity_x',
                  'velocity_y',
                  ]
    for attribute in attributes:
        pd[attribute] = pd[attribute] / pixels_per_unit
    
    return pd

def load_multiple_datasets_into_single_pandas_data_frame(filenames, sync_frames=None):
    '''
    filenames   - list of hdf5 files to load, full path name
    sync_frames - list of frames, one for each filename, these sync_frames will all be set to zero
                  defaults to using first frame for each dataset as sync

    '''

    pds = [load_data_as_pandas_dataframe_from_hdf5_file(filename) for filename in filenames]
    if sync_frames is None:
        sync_frames = [np.min(pd.frames) for pd in pds]
        
    for i, pd in enumerate(pds):
        pd.index -= sync_frames[i]
        pd.frames -= sync_frames[i]
    
    combined_pd = pandas.concat(pds)
    
    return combined_pd

def cull_short_trajectories(pd, min_length=4):
    key_length_dict = get_objid_lengths(pd)
    keys, lengths = zip(*key_length_dict.items())
    keys = list(keys)
    lengths = list(lengths)
    indices = np.where(np.array(lengths)>min_length)[0]    
    keys_ok = np.array(keys)[indices]
    
    culled_pd = pd.query('objid in @keys_ok')
    
    return culled_pd
    
def cull_trajectories_that_do_not_cover_much_ground(pd, min_distance_travelled=10):
    distance_travelled = pd.speed.groupby(pd.objid).agg('sum')
    indices = np.where(distance_travelled > min_distance_travelled)[0]
    objids = distance_travelled.index[indices]
    indices_where_object_acceptable = pd.objid.isin(objids)
    culled_pd = pd[indices_where_object_acceptable]
    return culled_pd
    
def cull_trajectories_that_do_not_cover_much_x_or_y_distance(pd, min_distance_travelled=10):
    min_x = pd.position_x.groupby(pd.objid).agg('min')
    max_x = pd.position_x.groupby(pd.objid).agg('max')
    distance_travelled = max_x - min_x
    indices = np.where(distance_travelled > min_distance_travelled)[0]
    objids = distance_travelled.index[indices]
    indices_where_object_acceptable = pd.objid.isin(objids)
    culled_pd = pd[indices_where_object_acceptable]
    pd = culled_pd 
    
    min_y = pd.position_y.groupby(pd.objid).agg('min')
    max_y = pd.position_y.groupby(pd.objid).agg('max')
    distance_travelled = max_y - min_y
    indices = np.where(distance_travelled > min_distance_travelled)[0]
    objids = distance_travelled.index[indices]
    indices_where_object_acceptable = pd.objid.isin(objids)
    culled_pd = pd[indices_where_object_acceptable]
    
    return culled_pd

def get_objid_lengths(pd, objid_attribute='objid'):
    keys = np.unique(pd[objid_attribute])
    lengths = np.bincount(pd[objid_attribute])
    
    true_lengths = lengths[np.nonzero(lengths)[0]]
    
    key_length_dict = dict(zip(keys,true_lengths))
    return key_length_dict
    
def remove_rows_above_speed_threshold(pd, speed_threshold=10):
    q = 'speed < ' + str(speed_threshold)
    return pd.query(q)
    
def remove_objects_that_never_exceed_minimum_speed(pd, speed_threshold=1):

    speeds = pd.speed.groupby(pd.objid).max()
    
    keysok = np.where(speeds.values > speed_threshold)
    objidsok = speeds.iloc[keysok].index
    pd_q = pd.query('objid in @objidsok')

    return pd_q






