
# coding: utf-8

# In[9]:

pwd # check your present working directory


# In[10]:

cd /home/caveman/catkin_ws/src/multi_tracker/sample_data/ # navigate to where your sample data is


# In[30]:

import multi_tracker_analysis as mta
import pandas
get_ipython().magic(u'matplotlib inline')
import matplotlib.pyplot as plt
import numpy as np


# In[13]:

pd = pandas.read_pickle('trackedobjects.pickle')


# In[48]:

# to get information on a function type the following, and info will appear at the bottom of the screen
get_ipython().magic(u'pinfo mta.plot.get_bins_from_backgroundimage')


# In[18]:

binsx, binsy = mta.plot.get_bins_from_backgroundimage('20151014_145627_N1_deltavideo_bgimg_20151014_1456.png')
mta.plot.plot_trajectories(pd, binsx, binsy)
plt.show()


# In[22]:

pd_subset = pd.query('speed > 1') # select rows where speed is larger than one, using pandas query function
mta.plot.plot_trajectories(pd_subset, binsx, binsy)
plt.show()


# In[43]:

keys, nframes = mta.data_slicing.get_nframes_per_key(pd)
longest_key_index = np.argmax(nframes)
longest_key = keys[longest_key_index]
print longest_key


# In[46]:

dataset = mta.read_hdf5_file_to_pandas.Dataset(pd)
trajec = dataset.trajec(49359)
fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(trajec.time_epoch, trajec.position_x)
    


# In[37]:

fig = plt.figure()
ax = fig.add_subplot(111)
bins = np.linspace(0,5,20)
result = ax.hist(pd.speed.values, bins=bins)
ax.set_xlabel('speed (pixels/sec)')
ax.set_ylabel('N occurences')


# In[52]:

# calculate frames where flies are in a particular region, save that to the pandas dataset as a new column
pd = mta.data_slicing.calc_frames_with_object_in_circular_region(pd, (495,245), 80, 'region_a')
pd.region_a


# In[56]:

pd_a = pd[pd.region_a == 1] # select only the rows of the dataset where the value of region_a is 1
pd_a.region_a


# In[55]:

# plot the subset!
mta.plot.plot_trajectories(pd_a, binsx, binsy)
plt.show()


# In[ ]:



