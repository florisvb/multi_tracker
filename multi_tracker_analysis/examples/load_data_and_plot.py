import multi_tracker_analysis as mta
import matplotlib.pyplot as plt

# load data from csv file as a python object
data = mta.read_csv_file_to_python.load_data_as_python_object_from_csv_file('data.csv')

# plot all the trajectories of length > 25
min_length = 25
mta.plot.plot_trajectories(data, min_length)
plt.show()
