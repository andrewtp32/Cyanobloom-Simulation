# This script is responsible for creating a list of 2 dimensional points. The two dimensional points represent
# positions of cyanoblooms within a simulated gazebo world. The simulated gazebo world contains a rectangular pond
# (dimensions of 300m x 200m). The idea is to create ten lists of points that correlate to the positions of
# cyanoblooms over a period of time. The velocity of the cyanoblooms will model real world bloom velocities under
# weather conditions (primarily wind).

import numpy as np
import random
import matplotlib.pyplot as plt

# x and y values for plotting the bloom positions
x_values = []
y_values = []
# 2 dimensional means for the gaussian distributions.
starting_means_evenly_spread_over_y = [[-165, -80], [-165, -40], [-165, 0], [-165, 40], [-165, 80]]
starting_means_bottom_half_of_y = [[-165, 0], [-165, -40], [-165, -75]]
starting_means_not_evenly_spread_over_x_and_y = [[-165, -60], [-140, -30], [-130, -50], [-120, 40], [-140, 25], [-115, -20]]
# Covariance for the starting bloom position uniform distributions
covariance = [[4, 0],
              [0, 4]]
# initialize the list for the starting bloom positions
bloom_positions_list = [[0, 0]]
count = 0
# generate cyanobloom points using the means in the starting_means list
for mean in starting_means_not_evenly_spread_over_x_and_y:
    distribution = np.random.multivariate_normal(mean, covariance, 15)
    bloom_positions_list = np.concatenate((bloom_positions_list, distribution), axis=0)
# remove the first point from the list. this point was (0, 0)
bloom_positions_list = bloom_positions_list[1:]


def east_to_west_with_noise(point):
    point[0] = point[0] + 30 + random.randint(-5, 5)
    point[1] = point[1] + random.randint(-5, 5)
    # following if statements force the blooms to stay within the 300x200 grid
    if point[0] > 150:
        point[0] = 150
    if point[0] < -150:
        point[0] = -150
    if point[1] > 100:
        point[1] = 100
    if point[1] < -100:
        point[1] = -100
    return point


def east_to_west_no_nose(point):
    point[0] = point[0] + 30
    point[1] = point[1]
    # following if statements force the blooms to stay within the 300x200 grid
    if point[0] > 150:
        point[0] = 150
    if point[0] < -150:
        point[0] = -150
    if point[1] > 100:
        point[1] = 100
    if point[1] < -100:
        point[1] = -100
    return point


def drift_north_east(point):
    point[0] = point[0] + 30
    point[1] = point[1] + 10
    # following if statements force the blooms to stay within the 300x200 grid
    if point[0] > 150:
        point[0] = 150
    if point[0] < -150:
        point[0] = -150
    if point[1] > 100:
        point[1] = 100
    if point[1] < -100:
        point[1] = -100
    return point


def drift_north_east_with_noise(point):
    point[0] = point[0] + 30 + random.randint(-5, 5)
    point[1] = point[1] + 10 + random.randint(-5, 5)
    # following if statements force the blooms to stay within the 300x200 grid
    if point[0] > 150:
        point[0] = 150
    if point[0] < -150:
        point[0] = -150
    if point[1] > 100:
        point[1] = 100
    if point[1] < -100:
        point[1] = -100
    return point


def drift_north_east_with_noise_with_reflection(point):
    point[0] = point[0] + 30 + random.randint(-5, 5)
    point[1] = point[1] + 10 + random.randint(-5, 5)
    # following if statements force the blooms to stay within the 300x200 grid
    x_boarder_neg = -150
    x_boarder_pos = 150
    y_boarder_neg = -75
    y_boarder_pos = 75
    if point[0] > x_boarder_pos:
        point[0] = (2 * x_boarder_pos) - point[0]
    if point[0] < x_boarder_neg:
        point[0] = (2 * x_boarder_neg) - point[0]
    if point[1] > y_boarder_pos:
        point[1] = (2 * y_boarder_pos) - point[1]
    if point[1] < y_boarder_neg:
        point[1] = (2 * y_boarder_neg) - point[1]
    return point


while count < 10:
    # Iterate through each point in the bloom position list and add to a new list
    new_bloom_position_list = [[0, 0]]
    for bloom in bloom_positions_list:
        shifted_point = drift_north_east_with_noise_with_reflection(bloom)
        # add the edited points to a new list
        new_bloom_position_list = np.concatenate((new_bloom_position_list, [shifted_point]), axis=0)

    # the previous points in bloom_positions_list are replaced by the current points provided by new_bloom_position_list
    bloom_positions_list = new_bloom_position_list[1:]

    file = open(
        f"/Users/andrewphillips/Desktop/bloom_coordinates_northeast_wind_drift_with_noise_and_reflection/bloom_coordinates_northeast_wind_drift_with_noise_and_reflection{count}.txt",
        "w")
    for v in bloom_positions_list:
        np.savetxt(file, [v], delimiter=' ')

    # x and y values for plotting the bloom positions
    x_values = []
    y_values = []
    for v in bloom_positions_list:
        x_values = np.append(x_values, v[0])
        y_values = np.append(y_values, v[1])
    plt.scatter(x_values, y_values)
    plt.axis([-160, 160, -110, 110])
    plt.title(f"bloom_coordinates_northeast_wind_drift{count}.txt")
    plt.grid()
    plt.show()
    plt.clf()

    count += 1
