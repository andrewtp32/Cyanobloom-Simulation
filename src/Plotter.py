import os
import numpy as np
import pandas as pd
from numpy import genfromtxt
import matplotlib.pyplot as plt


def convert_array_to_cartesian(global_coordinate_arr, center_point):
    # move to origin
    arr_at_origin = global_coordinate_arr - center_point
    # scale to meters and return
    return arr_at_origin * 111139


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
# reformat the coordinates
month_coordinates_arr = convert_array_to_cartesian(month_coordinates_arr, [-70.09755158499999, 44.145137175])

""" --------------- Line graphs of the particle population at each step --------------- """
# create a plot
fig1, ax1 = plt.subplots()

# initialize the lists
pop_amount_arr = []

# read the txt files
for filename in filename_list:
    # load the text file
    ground_truth_scatter = genfromtxt(f'{directory}/{filename}', delimiter=',')
    # record the amount of particles at this time step
    pop_amount_arr.append(len(ground_truth_scatter))

# plot
ax1.plot(pop_amount_arr)
# configure the x-labels
ax1.set_xticks([0, 744, 1464, 2208, 2952, 3672], ['May', 'June', 'July', 'Aug', 'Sep', 'Oct'])

""" --------------- Plot of all the GTs on the lake --------------- """
# create plot
fig0, ax0 = plt.subplots()
ax0.set_aspect('equal')

# read the txt files
for index, filename in enumerate(filename_list[2000:]):
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
    ax0.set_title(f"{filename} - {len(ground_truths_x)}")
    ax0.scatter(month_coordinates_arr_x, month_coordinates_arr_y, color="blue")
    ax0.scatter(ground_truths_x, ground_truths_y, color="green", s=3)
    plt.pause(0.01)
    plt.cla()
