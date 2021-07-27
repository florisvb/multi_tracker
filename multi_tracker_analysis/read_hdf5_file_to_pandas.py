import numpy as np
import h5py
import copy
import pandas
import os
import imp
import pickle
import scipy.interpolate
import warnings
import time
import matplotlib.pyplot as plt
import inspect
import types
import warnings

def get_filenames(path, contains, does_not_contain=['~', '.pyc']):
    cmd = 'ls ' + '"' + path + '"'
    ls = os.popen(cmd).read()
    all_filelist = ls.split('\n')
    try:
        all_filelist.remove('')
    except:
        pass
    filelist = []
    for i, filename in enumerate(all_filelist):
        if contains in filename:
            fileok = True
            for nc in does_not_contain:
                if nc in filename:
                    fileok = False
            if fileok:
                filelist.append( os.path.join(path, filename) )
    return filelist
    
def get_filename(path, contains, does_not_contain=['~', '.pyc']):
    filelist = get_filenames(path, contains, does_not_contain)
    if len(filelist) == 1:
        return filelist[0]
    elif len(filelist) > 0 and 'bgimg' in contains:
        pick = sorted(filelist)[-1]
        print('Found multiple background images, using ' + str(pick))
        return pick
    else:
        print (filelist)
        print ('Found too many, or too few files')
    return None
            
def load_bag_as_hdf5(bag, skip_messages=[]):
    output_fname = bag.split('.')[0] + '.hdf5'
    print (output_fname)
    if not os.path.exists(output_fname):
        mta.bag2hdf5.bag2hdf5(   bag,
                                 output_fname,
                                 max_strlen=200,
                                 skip_messages=skip_messages)    
    metadata = h5py.File(output_fname, 'r')
    return metadata

class Trajectory(object):
    def __init__(self, pd, objid, functions=None):
        self.pd = pd[pd['objid']==objid]
        for column in self.pd.columns:
            self.__setattr__(column, self.pd[column].values)
        if functions is not None:
            self.__attach_analysis_functions__(functions)

            
    def __getitem__(self, key): # trajec attributes can be accessed just like a dictionary this way
        return self.__getattribute__(key)

class Dataset(object):
    def __init__(self, pd, path=None, save=False, convert_to_units=False, annotations=None):
        '''
        highly recommended to provide directory path
        
        convert_to_units requires that path is given, and that path contains a config file, which has attributes:
            - pixels_per_mm (or pixels_per_cm, etc) 
            - position_zero = [x, y] # the x and y pixels of position zero
            - frames_per_second defined
        '''
        self.pd = pd
        self.keys = []
        self.__processed_trajecs__ = {}
        self.save = save
        self.path = path
        self.annotations = annotations

        self.units = {'length': 'pixels', 'speed': 'pixels per frame'}
        if path is not None:
            if convert_to_units:
                self.load_config()
                pixels_per_unit_key = []
                for key in self.config.__dict__.keys():
                    if 'pixels_per_' in key:
                        pixels_per_unit_key = key
                        self.units['length'] = key.split('pixels_per_')[1]
                        self.units['speed'] = self.units['length'] + ' per second'
                        break
                self.pixels_per_unit = self.config.__dict__[pixels_per_unit_key]
                self.frames_per_second = self.config.frames_per_second
                self.convert_to_units()
            
            self.set_dataset_filename()

        if save:
            self.load_keys()
            self.copy_trajectory_objects_to_dataset()
            del(self.pd)
            self.pd = None
            print()
            print ('Dataset loaded as a stand alone object - to save your dataset, use: ')
            print ('dataset.save_dataset()')
            print()
            print ('  -- OR --  ')
            print()
            print ('del (dataset.config)')
            print ('import pickle')
            print ('f = open(dataset.dataset_filename, "w+")')
            print( 'pickle.dump(dataset, f)')
            print ('f.close()')

    def set_dataset_filename(self):
        raw_data_filename = get_filename(self.path, 'trackedobjects.hdf5')
        self.dataset_filename = raw_data_filename.split('trackedobjects.hdf5')[0] + 'trackedobjects_dataset.pickle'

    def convert_to_units(self):
        self.pd.position_x = (self.pd.position_x-self.config.position_zero[0])/float(self.pixels_per_unit)
        self.pd.position_y = (self.pd.position_y-self.config.position_zero[1])/float(self.pixels_per_unit)
        self.pd.speed = self.pd.speed/float(self.pixels_per_unit)*self.frames_per_second
        self.pd.velocity_x = self.pd.velocity_x/float(self.pixels_per_unit)*self.frames_per_second
        self.pd.velocity_y = self.pd.velocity_y/float(self.pixels_per_unit)*self.frames_per_second

    def load_config(self):
        self.config = load_config_from_path(self.path)

    def save_dataset(self):
        try:
            del(self.config)
        except:
            pass
        f = open(self.dataset_filename, "w+")
        pickle.dump(self, f)
        f.close()

    def trajec(self, key):
        if self.pd is not None:
            trajec = Trajectory(self.pd, key)
            return trajec
        else:
            return self.trajecs[key]
            #raise ValueError('This is a saved dataset, use dict access: Dataset.trajecs[key] for data')


    def framestamp_to_timestamp(self, frame):
        t = self.pd.ix[frame]['time_epoch']
        try:
            return t.iloc[0]
        except:
            return t
            
    def timestamp_to_framestamp(self, t):
        first_time = self.pd['time_epoch'].values[0]
        first_frame = self.pd['frames'].values[0]
        last_time = self.pd['time_epoch'].values[-1]
        last_frame = self.pd['frames'].values[-1]
        func = scipy.interpolate.interp1d([first_time, last_time],[first_frame, last_frame])
        return int(func(t))
        
    def load_keys(self, keys=None):
        if self.annotations is None:
            if keys is None:
                self.keys = np.unique(self.pd.objid).tolist()
            else:
                self.keys = keys
        else:
            self.keys = []
            for key, note in self.annotations.items():
                if 'confirmed' in note['notes']:
                    self.keys.append(key)

    def copy_trajectory_objects_to_dataset(self):
        self.trajecs = {}
        for key in self.keys:
            trajec = copy.copy( Trajectory(self.pd, key) )
            self.trajecs.setdefault(key, trajec)

    def calculate_function_for_all_trajecs(self, function):
        for key, trajec in self.trajecs.items():
            function(trajec)

    def remove_zero_length_objects(self):
        if 'trajecs' in self.__dict__:
            for key, trajec in self.trajecs.items():    
                if len(trajec.speed) == 0:
                    try: 
                        del(self.trajecs[key]) 
                    except: 
                        pass
                    try: 
                        self.keys.remove(key) 
                    except: 
                        pass
            for key in self.keys:
                if key not in self.trajecs.keys():
                    self.keys.remove(key)
        else:
            warnings.warn('remove zero length objects only works on copyied datasets')

    def has_zero_length_objects(self):
        if 'trajecs' in self.__dict__:
            for key, trajec in self.trajecs.items():    
                if len(trajec.speed) == 0:
                    return True
            for key in self.keys:
                if key not in self.trajecs.keys():
                    return True
            return False
        else:
            warnings.warn('remove zero length objects only works on copyied datasets')

def load_dataset_from_path(path, load_saved=False, convert_to_units=True, use_annotations=True):
    '''
    load_saved only recommended for reasonably sized datasets, < 500 mb
    convert_to_units - see Dataset; converts pixels and frames to mm (or cm) and seconds, based on config
    '''
    if load_saved:
        data_filename = get_filename(path, 'trackedobjects_dataset.pickle')
        if data_filename is not None:
            print (data_filename)
            delete_cut_join_instructions_filename = get_filename(path, 'delete_cut_join_instructions.pickle')
            epoch_time_when_dcjif_modified = os.path.getmtime(delete_cut_join_instructions_filename)
            epoch_time_when_dataset_modified = os.path.getmtime(data_filename)

            if epoch_time_when_dcjif_modified > epoch_time_when_dataset_modified:
                print ('Delete cut join instructions modified - recalculating new dataset')

            else:
                f = open(data_filename)
                dataset = pickle.load(f)
                f.close()
                # check path
                if dataset.path != path: # an issue if the files get moved around
                    dataset.path = path 
                    dataset.set_dataset_filename()
                if dataset.has_zero_length_objects():
                    dataset.remove_zero_length_objects()
                    dataset.save_dataset()
                print ('Loaded cached dataset last modified: ')
                print (time.localtime(epoch_time_when_dataset_modified))
                print()
                return dataset
        else:
            print ('Could not find cached dataset in path: ')
            print (path)
            print (' Loading dataset from raw data now...')

    data_filename = get_filename(path, 'trackedobjects.hdf5')
    pd, config = load_and_preprocess_data(data_filename)

    if use_annotations:
        annotations_file = open(get_filename(path, 'annotations'))
        annotations = pickle.load(annotations_file)
        annotations_file.close()
    else:
        annotations = None

    dataset = Dataset(pd, path=path, 
                          save=load_saved, 
                          convert_to_units=convert_to_units,
                          annotations=annotations) # if load_saved is True, copy the dataset, so it can be cached

    if load_saved:
        dataset.remove_zero_length_objects()
        dataset.save_dataset()

    return dataset
            
def load_data_as_pandas_dataframe_from_hdf5_file(filename, attributes=None):
    if '.pickle' in filename:
        pd = pandas.read_pickle(filename)
        return pd

    try:
        data = h5py.File(filename, 'r', swmr=True)['data']
    except ValueError:
        data = h5py.File(filename, 'r', swmr=False)['data']

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
    #pd = pd.drop(pd.index==[0]) # delete 0 frames (frames with no data)
    pd = pd[pd.frames!=0] # new pandas doesn't work with above line
    pd = calc_additional_columns(pd)
    # pd_subset = pd[pd.objid==key]
    return pd
    
def load_and_preprocess_data(hdf5_filename):
    '''
    requires that a configuration file be found in the same directory as the hdf5 file, with the same prefix
    
    returns: pandas dataframe, processed according to configuration file, and the configuration file instance
    '''

    if 'trackedobjects' not in hdf5_filename:
        print ('File is not a trackedobjects file, looking for a trackedobjects file in this directory')
        fname = get_filename(hdf5_filename, 'trackedobjects.hdf5')
        if fname is not None:
            hdf5_filename = fname
            print ('Found: ', fname)
        else:
            raise ValueError('Could not find trackedobjects.hdf5 file')
                    
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

def load_config_from_path(path):
    config_filename = get_filename(path, 'config')
    try:
        hdf5_file = os.path.basename(get_filename(path, 'trackedobjects.hdf5'))
        identifiercode = hdf5_file.split('_trackedobjects')[0]
    except:
        config_file_basename = os.path.basename(config_filename)
        identifiercode = config_file_basename.split('config_')[1].split('.py')[0]
    print ('identifiercode: ', identifiercode)
    if config_filename is not None:
        Config = imp.load_source('Config', config_filename)
        config = Config.Config(path, identifiercode)
    else:
        config = None
    return config

def load_data_selection_from_path(path):
    filename = get_filename(path, contains='dataframe_')
    pd = pandas.read_pickle(filename)
    config = load_config_from_path(os.path.dirname(path))
    return pd, config
    
def find_instructions_related_to_objid(instructions, objid):
    for i, instruction in enumerate(instructions):
        if 'new_objid' in instruction.keys():
            if objid == instruction['new_objid']:
                print (i)
        if 'objids' in instruction.keys():
            if objid in instruction['objids']:
                print (i)

def mass_delete(pd, objids_to_delete):
    print('Mass deleting objects as requested...')
    pd = pd[~pd['objid'].isin(objids_to_delete)]
    return pd
    
def delete_cut_join_trajectories_according_to_instructions(pd, instructions, interpolate_joined_trajectories=True):
    if type(instructions) is str:
        f = open(instructions)
        instructions = pickle.load(f)
        f.close()
    elif type(instructions) is not list:
        instructions = [instructions]
    
    def get_proper_order_of_objects(dataset, keys):
        trajecs = []
        ts = []
        goodkeys = []
        for key in keys:
            trajec = dataset.trajec(key)
            if len(trajec.speed) > 0:
                trajecs.append(trajec)
                ts.append(trajec.time_epoch[0])
                goodkeys.append(key)
        order = np.argsort(ts)
        return np.array(goodkeys)[order]
        
    def get_indices_to_use_for_interpolation(key1, key2):
        length_key1 = len(dataset.trajec(key1).position_x)
        first_index_key1 = np.max( [length_key1-4, 0] )
        indices_key1 = np.arange( first_index_key1, length_key1 )
        
        length_key2 = len(dataset.trajec(key2).position_x)
        last_index_key2 = np.min( [length_key2, 0+4] )
        indices_key2 = np.arange( 0, last_index_key2 )
    
        return indices_key1, indices_key2
    
    for instruction in instructions:
        if instruction['action'] == 'delete':
            #pass
            if type(instruction['objid']) == list:
                pd = mass_delete(pd, instruction['objid'])
            else:
                pd = pd[pd.objid!=instruction['objid']]
                
        elif instruction['action'] == 'cut':
            mask = (pd['objid']==instruction['objid']) & (pd['frames']>instruction['cut_frame_global'])
            pd.loc[mask,'objid'] = instruction['new_objid']
        elif instruction['action'] == 'join':
            if interpolate_joined_trajectories is False:
                for key in instruction['objids']:
                    mask = pd['objid']==key
                    if 'new_objid' in instruction.keys():
                        print ('*** ASSIGNING NEW OBJID: ', instruction['new_objid'])
                        pd.loc[mask,'objid'] = instruction['new_objid']
                    else:
                        warnings.warn("Warning: using old join method; not using unique objid numbers")
                        pd.loc[mask,'objid'] = instruction['objids'][0]
            elif interpolate_joined_trajectories is True:
                dataset = Dataset(pd)
                keys = get_proper_order_of_objects(dataset, instruction['objids'])
                for k, key in enumerate(keys[0:-1]):
                    dataset = Dataset(pd)
                    last_frame = dataset.trajec(keys[k]).frames[-1]
                    first_frame = dataset.trajec(keys[k+1]).frames[0]
                    if first_frame <= last_frame: # overlap between objects, keep the second object's data, since the first is likely bad kalman projections
                        mask = np.invert( (pd['objid']==keys[k]) & (pd['frames']>=first_frame) )
                        pd = pd[mask]
                    else:
                        frames_to_interpolate = np.arange(last_frame+1, first_frame)
                        if len(frames_to_interpolate) > 0:
                            indices_key1, indices_key2 = get_indices_to_use_for_interpolation(keys[k], keys[k+1])
                            x = np.hstack((dataset.trajec(keys[k]).frames[indices_key1], dataset.trajec(keys[k+1]).frames[indices_key2]))
                            new_pd_dict = {attribute: None for attribute in pd.columns}
                            new_pd_dict.setdefault('interpolated', None)
                            index = frames_to_interpolate
                            
                            if 'data_to_add' in instruction.keys():
                                data_to_add_frames = []
                                data_to_add_x = []
                                data_to_add_y = []
                                for index, data_to_add in enumerate(instruction['data_to_add']):
                                    frame_for_data_to_add = dataset.timestamp_to_framestamp(data_to_add[0])
                                    print( frame_for_data_to_add, last_frame, first_frame)
                                    if frame_for_data_to_add > last_frame and frame_for_data_to_add < first_frame:
                                        data_to_add_frames.append(frame_for_data_to_add)
                                        data_to_add_x.append(data_to_add[1])
                                        data_to_add_y.append(data_to_add[2])
                                order = np.argsort(data_to_add_frames)
                                data_to_add_frames = np.array(data_to_add_frames)[order]
                                data_to_add_x = np.array(data_to_add_x)[order]
                                data_to_add_y = np.array(data_to_add_y)[order]
                            
                            for attribute in pd.columns:
                                if attribute == 'objid':
                                    attribute_values = [keys[0] for f in frames_to_interpolate]
                                elif attribute == 'frames':
                                    attribute_values = frames_to_interpolate
                                else:
                                    y = np.hstack((dataset.trajec(keys[k])[attribute][indices_key1], dataset.trajec(keys[k+1])[attribute][indices_key2]))  
                                    
                                    if 'data_to_add' in instruction.keys():
                                        if 'position' in attribute:
                                            x_with_added_data = np.hstack((x, data_to_add_frames))
                                            if attribute == 'position_x':
                                                y_with_added_data = np.hstack((y, data_to_add_y))   
                                            elif attribute == 'position_y':
                                                y_with_added_data = np.hstack((y, data_to_add_x))  
                                            order = np.argsort(x_with_added_data)
                                            x_with_added_data = x_with_added_data[order]
                                            y_with_added_data = y_with_added_data[order]
                                            func = scipy.interpolate.interp1d(x_with_added_data,y_with_added_data)
                                        else:
                                            func = scipy.interpolate.interp1d(x,y)
                                    else:
                                        func = scipy.interpolate.interp1d(x,y)
                                    
                                    attribute_values = func(frames_to_interpolate)
                                new_pd_dict[attribute] = attribute_values
                            interpolated_values = np.ones_like(new_pd_dict['position_x'])
                            new_pd_dict['interpolated'] = interpolated_values
                            #return pd, new_pd_dict, frames_to_interpolate
                            new_pd = pandas.DataFrame(new_pd_dict, index=frames_to_interpolate)
                            pd = pandas.concat([pd, new_pd])
                            pd = pd.sort_index()
                for key in instruction['objids']:
                    mask = pd['objid']==key
                    if 'new_objid' in instruction.keys():
                        print ('*** ASSIGNING NEW OBJID: ', key, '  to : ', instruction['new_objid'])
                        pd.loc[mask,'objid'] = instruction['new_objid']
                    else:
                        warnings.warn("Warning: using old join method; not using unique objid numbers")
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

def compare_objids_from_two_dataframes(pd1, pd2):
    objids_1 = np.unique(pd1.objid.values)
    objids_2 = np.unique(pd2.objid.values)
    
    unique_to_1 = [k for k in objids_1 if k not in objids_2]
    unique_to_2 = [k for k in objids_2 if k not in objids_1]
    
    return unique_to_1, unique_to_2
    
def cull_trajectories_that_do_not_cover_much_ground(pd, min_distance_travelled=10, print_keys=False):
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






