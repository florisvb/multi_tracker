import imp
import os
import numpy as np
import multi_tracker_analysis as mta

class Config(object):
    def __init__(self, path, identifiercode=''):
        if '.py' in path:
            self.path = os.path.dirname(path)
        else:
            self.path = path
        self.identifiercode = identifiercode
        
        # pre-processing data parameters
        self.preprocess_data_function = self.preprocess_data
        self.minlength = 5 # in frames
        self.maxspeed = 10 # pixels / frame
        self.minspeed = 0
        self.minimal_cumulative_distance_travelled = 4
        
        # other parameters
        self.sensory_stimulus_on = []
        
    def preprocess_data(self, pandas_dataframe):
        print 'Preprocessing data - see config file for details!'
        
        pandas_dataframe = mta.read_hdf5_file_to_pandas.cull_short_trajectories(pandas_dataframe, self.minlength)
        pandas_dataframe = mta.read_hdf5_file_to_pandas.remove_rows_above_speed_threshold(pandas_dataframe, speed_threshold=self.maxspeed)
        pandas_dataframe = mta.read_hdf5_file_to_pandas.remove_objects_that_never_exceed_minimum_speed(pandas_dataframe, speed_threshold=self.minspeed)
        pandas_dataframe = mta.read_hdf5_file_to_pandas.cull_trajectories_that_do_not_cover_much_x_or_y_distance(pandas_dataframe, min_distance_travelled=self.minimal_cumulative_distance_travelled)
        
        # Delete cut and join trajectories
        instructions_filename = mta.read_hdf5_file_to_pandas.get_filename(self.path, 'delete_cut_join_instructions.pickle')
        if instructions_filename is not None:
            pandas_dataframe = mta.read_hdf5_file_to_pandas.delete_cut_join_trajectories_according_to_instructions(pandas_dataframe, instructions_filename)
        else:
            print 'No delete cut join instructions found in path!'
        
        return pandas_dataframe
        
