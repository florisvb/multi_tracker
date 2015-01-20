import multi_tracker_analysis as mta
import matplotlib.pyplot as plt

data = mta.read_csv_file_to_python.load_data_as_python_object_from_csv_file('/home/brainhacker/multi_tracker_data_files/data.csv' )
length = 50
mta.plot.plot_trajectories(data, length)
plt.show()
