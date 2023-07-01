import os
import numpy as np
import pandas as pd
from numpy import genfromtxt
import matplotlib.pyplot as plt

# assign the directory of all the resultant files
directory = f"{(os.getcwd())[:-4]}/ground_truths/SabattusPond/2016"
# initialize a list of filename strings
filename_list = []
# iterate through each file in the specified directory. append the filename to filename_list
for file in os.listdir(directory):
    filename_list.append(os.fsdecode(file))
# sort the list to be in alphabetical order
filename_list = sorted(filename_list)

# read the csv
full_df = pd.read_csv(f'{(os.getcwd())[:-4]}/jess_sat_data/hotspots_monthyear.csv')
# locate relevant satellite data
month_df = full_df.loc[(full_df['Year'] == 2016) & (full_df['Month'] == 5)]
# pull out just the coordinates
month_df_coordinates = month_df[['x', 'y']].copy()
month_df_coordinates.to_numpy()
month_coordinates_arr = np.array(month_df_coordinates)

""" --------------- Plot of all the GTs on the lake --------------- """
# create plot
fig, ax = plt.subplots(figsize=[6, 8])
ax.set_aspect('equal')

# read the txt files
for index, filename in enumerate(filename_list):
    ground_truths_x = []
    ground_truths_y = []
    # load the text file
    ground_truth_scatter = genfromtxt(f'{directory}/{filename}', delimiter=',')
    if len(ground_truth_scatter) > 0:
        # unzip GT
        ground_truths_x, ground_truths_y = zip(*ground_truth_scatter)
    # unzip GT
    month_coordinates_arr_x, month_coordinates_arr_y = zip(*month_coordinates_arr)
    # create figure for each time step
    ax.set_title(f"{filename} - {len(ground_truths_x)}")
    ax.scatter(month_coordinates_arr_x, month_coordinates_arr_y, color="blue")
    ax.scatter(ground_truths_x, ground_truths_y, color="green", s=1)
    plt.pause(0.05)
    plt.cla()
