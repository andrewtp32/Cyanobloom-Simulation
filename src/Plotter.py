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
# scale down by a factor of 10
# month_coordinates_arr = month_coordinates_arr / 10

# save water boundary points
with open(f"{(os.getcwd())[:-4]}/jess_sat_data/water_boundary_points_scaled.txt", "w") as gt_file:
    # write the values to the file
    np.savetxt(gt_file, month_coordinates_arr[::3], '%f')

""" --------------- Line graphs of the particle population at each step --------------- 
# create a plot
fig1, ax1 = plt.subplots()

# initialize the lists
pop_amount_arr = []

# read the txt files
for filename in filename_list:
    # load the text file
    ground_truth_scatter = genfromtxt(f'{directory}/{filename}', delimiter=',', skip_header=1)
    try:
        # record the amount of particles at this time step
        pop_amount_arr.append(len(ground_truth_scatter))
    except TypeError:
        # if there is an error with the length of ground_truth_scatter, then just append the number 0
        pop_amount_arr.append(0)

# plot
ax1.plot(pop_amount_arr)
# configure the x-labels and grid
month_ticks = [0, 744, 1464, 2208, 2952, 3672]
minor_ticks = [0, 186, 372, 558, 744,
               924, 1104, 1284, 1464,
               1650, 1836, 2022, 2208,
               2394, 2580, 2766, 2952,
               3132, 3312, 3492, 3672,
               3858, 4044, 4230, 4414]
ax1.set_xticks(month_ticks, ['May', 'June', 'July', 'Aug', 'Sep', 'Oct'])
ax1.set_xticks(minor_ticks, minor=True)

# And a corresponding grid+2
ax1.grid(which='both')
# Or if you want different settings for the grids:
ax1.grid(which='minor', alpha=0.2)
ax1.grid(which='major', alpha=0.5)
plt.show()
"""

""" --------------- Plot of all the GTs on the lake --------------- """

# create plot
fig0, ax0 = plt.subplots()
ax0.set_aspect('equal')

# read the txt files
for index, filename in enumerate(filename_list[::24]):
    ground_truths_x = []
    ground_truths_y = []
    # load the text file
    # with open(f'{directory}/{filename}') as file:
    #     for line in file:
    #         ground_truths_x.append(line[0])
    #         ground_truths_y.append(line[1])
    # print(ground_truths_x)
    ground_truth_scatter = genfromtxt(f'{directory}/{filename}', delimiter=' ', skip_header=1)
    if len(ground_truth_scatter) > 10:
        # unzip GT
        ground_truths_x, ground_truths_y = zip(*ground_truth_scatter)
    # unzip GT
    month_coordinates_arr_x, month_coordinates_arr_y = zip(*month_coordinates_arr)
    # create figure for each time step
    ax0.set_title(f"{filename} - {len(ground_truths_x)}")
    ax0.scatter(month_coordinates_arr_x, month_coordinates_arr_y, color="blue")
    ax0.scatter(ground_truths_x, ground_truths_y, color="green", s=3)
    plt.pause(10)
    plt.cla()
