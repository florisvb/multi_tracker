import numpy as np
import matplotlib.pyplot as plt
import multi_tracker_analysis as mta
import fly_plot_lib.flymath as flymath

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

    chunks, indices = flymath.get_continuous_chunks(trajec.walking, array2=None, jump=0.5, return_index=False)

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