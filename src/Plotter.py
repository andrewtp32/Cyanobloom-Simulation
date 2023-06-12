import os
import numpy as np
import pandas as pd
from numpy import genfromtxt
import matplotlib.pyplot as plt

# assign the directory of all the resultant files
directory = "/Users/andrewphillips/Documents/Documents - Andrew’s MacBook Air/College/PaoloLab/uri_soft_wip/cyanobloom_simulation/ground_truths/SabattusPond/2016/ground_truth_coordinates"
# initialize a list of filename strings
filename_list = []
# iterate through each file in the specified directory. append the filename to filename_list
for file in os.listdir(directory):
    filename_list.append(os.fsdecode(file))
# sort the list to be in alphabetical order
filename_list = sorted(filename_list)

# read excel file
data = pd.read_csv(
    "/Users/andrewphillips/Documents/Documents - Andrew’s MacBook Air/College/PaoloLab/uri_soft_wip/cyanobloom_simulation/jess_sat_data/2016/data_2016_5.csv")

# create plot
fig, ax = plt.subplots(figsize=[6, 8])
ax.set_aspect('equal')

# read the txt files
for index, filename in enumerate(filename_list[10:]):
    ground_truths_x = []
    ground_truths_y = []
    # use the file path from the user input GUI
    ground_truth_scatter = genfromtxt(f'{directory}/{filename}', delimiter=',')
    if len(ground_truth_scatter) > 0:
        # create independent x and y lists for each array
        ground_truths_x, ground_truths_y = zip(*ground_truth_scatter)
    # create figure for each time step
    ax.set_title(f"Time step: {filename[:-4]}")
    ax.scatter(data['x'], data['y'], color="blue")
    ax.scatter(ground_truths_x, ground_truths_y, color="green", s=0.5)
    plt.pause(0.5)
    plt.cla()
