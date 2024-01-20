import numpy as np
import matplotlib.pyplot as plt
import multi_tracker_analysis as mta

def get_continuous_chunks(array, array2=None, jump=1, return_index=False):
    """
    Splits array into a list of continuous chunks. Eg. [1,2,3,4,5,7,8,9] becomes [[1,2,3,4,5], [7,8,9]]
    
    array2  -- optional second array to split in the same way array is split
    jump    -- specifies size of jump in data to create a break point
    """
    diffarray = diffa(array)
    break_points = np.where(np.abs(diffarray) > jump)[0]
    break_points = np.insert(break_points, 0, 0)
    break_points = np.insert(break_points, len(break_points), len(array))
    
    chunks = []
    array2_chunks = []
    index = []
    for i, break_point in enumerate(break_points):
        if break_point >= len(array):
            break
        chunk = array[break_point:break_points[i+1]]
        if type(chunk) is not list:
            chunk = chunk.tolist()
        chunks.append(chunk)
        
        if array2 is not None:
            array2_chunk = array2[break_point:break_points[i+1]]
            if type(array2_chunk) is not list:
                array2_chunk = array2_chunk.tolist()
            array2_chunks.append(array2_chunk)
        
        if return_index:
            indices_for_chunk = np.arange(break_point,break_points[i+1])
            index.append(indices_for_chunk)
            
    if type(break_points) is not list:
        break_points = break_points.tolist()
        
    if return_index:
        return index
    
    if array2 is None:
        return chunks, break_points
    
    else:
        return chunks, array2_chunks, break_points

def plot(trajec, ax=None, x='position_x', y='position_y', linestyle='-'):

    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(trajec.__dict__[x], trajec.__dict__[y], linestyle=linestyle)

def calculate_stopped_and_walking_frames(trajec):

    stopped_threshold = 1 # mm/sec
    walking_threshold = 2 # mm/sec
    minimum_stop_length = 6 # frames, = 0.2 sec at 30 fps

    trajec.walking = np.zeros_like(trajec.position_x)

    for frame in range(len(trajec.walking)):
        if trajec.speed[frame] < stopped_threshold:
            trajec.walking[frame] = 0
        elif trajec.speed[frame] > walking_threshold:
            trajec.walking[frame] = 1
        else:
            try:
                trajec.walking[frame] = trajec.walking_threshold[frame-1]
            except:
                trajec.walking[frame] = 0

    chunks, indices = get_continuous_chunks(trajec.walking, array2=None, jump=0.5, return_index=False)

    for c, chunk in enumerate(chunks):
        ind = np.arange(indices[c], indices[c]+len(chunk))
        if len(ind) <= 10 and np.median(trajec.walking[ind])==0:
            trajec.walking[ind] = 1
        elif len(ind) <= 10 and np.median(trajec.walking[ind])==1:
            trajec.walking[ind] = 1

def calculate_fraction_of_time_spent_walking(trajec):
    if 'walking' not in trajec.__dict__.keys():
        calculate_stopped_and_walking_frames(trajec)
    trajec.fraction_of_time_spent_walking = np.mean(trajec.walking)

def calculate_mean_walking_speed_while_walking(trajec):
    if 'walking' not in trajec.__dict__.keys():
        calculate_stopped_and_walking_frames(trajec)
    trajec.mean_walking_speed = np.mean(trajec.speed) # np.mean(trajec.speed[np.where(trajec.walking)])
