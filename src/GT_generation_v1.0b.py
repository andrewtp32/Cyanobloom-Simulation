import os
import math
import numpy as np
import pandas as pd


def historical_probs_and_pop_and_neighbor_pop(t0_prob, t0_pop, t1_prob, t1_pop, t0_surrounding_cells):
    # calculate a random float between 0 and 1
    rand = np.random.uniform(0, 1, 1)[0]
    # find the difference in probabilities
    diff = t1_prob - t0_prob
    # multiply t1_prob by a limiting constant
    constant = 0.5
    # boolean for whether there is population is present in surround cells
    is_there_pop_in_surrounding_cells = False
    # loop through the neighboring cells
    for cell in t0_surrounding_cells:
        # check if there is any population in the cell
        if cell[3] > 0:
            is_there_pop_in_surrounding_cells = True

    if t1_prob > 0.7:
        if diff < -0.4:
            # proportioned change or chance to pop birth in empty cell with neighbor
            if t0_pop == 0 and t1_prob >= rand and is_there_pop_in_surrounding_cells:
                t1_pop = 1
            else:
                t1_pop = t0_pop * (1 + constant * t1_prob)
        if -0.4 <= diff < -0.15:
            # proportioned change or heightened chance to pop birth in empty cell with neighbor
            if t0_pop == 0 and t1_prob * 1.5 >= rand and is_there_pop_in_surrounding_cells:
                t1_pop = 1
            else:
                t1_pop = t0_pop * (1 + constant * t1_prob)
        if -0.15 <= diff <= 0.15:
            # very large change or pop birth in empty cell
            if t0_pop == 0:
                t1_pop = 1
            else:
                t1_pop = t0_pop * 2
        if 0.15 < diff <= 0.4:
            # proportioned change or heightened chance to pop birth in empty cell with neighbor
            if t0_pop == 0 and t1_prob * 1.5 >= rand and is_there_pop_in_surrounding_cells:
                t1_pop = 1
            else:
                t1_pop = t0_pop * (1 + constant * t1_prob)
        if diff > 0.4:
            # proportioned change or chance to pop birth in empty cell with neighbor
            if t0_pop == 0 and t1_prob >= rand and is_there_pop_in_surrounding_cells:
                t1_pop = 1
            else:
                t1_pop = t0_pop * (1 + constant * t1_prob)

    if 0.45 < t1_prob <= 0.7:
        if diff < -0.4:
            # proportioned change or limited chance to pop birth in empty cell with neighbor
            if t0_pop == 0 and t1_prob * 0.5 >= rand and is_there_pop_in_surrounding_cells:
                t1_pop = 1
            else:
                t1_pop = t0_pop * (1 + constant * t1_prob)
        if -0.4 <= diff < -0.15:
            # proportioned change or chance to pop birth in empty cell with neighbor
            if t0_pop == 0 and t1_prob >= rand and is_there_pop_in_surrounding_cells:
                t1_pop = 1
            else:
                t1_pop = t0_pop * (1 + constant * t1_prob)
        if -0.15 <= diff <= 0.15:
            # proportioned change or heightened chance to pop birth in empty cell with neighbor
            if t0_pop == 0 and t1_prob * 1.5 >= rand and is_there_pop_in_surrounding_cells:
                t1_pop = 1
            else:
                t1_pop = t0_pop * (1 + constant * t1_prob)
        if 0.15 < diff <= 0.4:
            # proportioned change or chance to pop birth in empty cell with neighbor
            if t0_pop == 0 and t1_prob >= rand and is_there_pop_in_surrounding_cells:
                t1_pop = 1
            else:
                t1_pop = t0_pop * (1 + constant * t1_prob)
        if diff > 0.4:
            # proportioned change or limited chance to pop birth in empty cell with neighbor
            if t0_pop == 0 and t1_prob * 0.5 >= rand and is_there_pop_in_surrounding_cells:
                t1_pop = 1
            else:
                t1_pop = t0_pop * (1 + constant * t1_prob)

    if 0.2 < t1_prob <= 0.45:
        if diff < -0.4:
            # proportioned change
            t1_pop = t0_pop * (1 + constant * t1_prob)
        if -0.4 <= diff < -0.15:
            # proportioned change or limited chance to pop birth in empty cell with neighbor
            if t0_pop == 0 and t1_prob * 0.5 >= rand and is_there_pop_in_surrounding_cells:
                t1_pop = 1
            else:
                t1_pop = t0_pop * (1 + constant * t1_prob)
        if -0.15 <= diff <= 0.15:
            # proportioned change or chance to pop birth in empty cell with neighbor
            if t0_pop == 0 and t1_prob >= rand and is_there_pop_in_surrounding_cells:
                t1_pop = 1
            else:
                t1_pop = t0_pop * (1 + constant * t1_prob)
        if 0.15 < diff <= 0.4:
            # proportioned change or limited chance to pop birth in empty cell with neighbor
            if t0_pop == 0 and t1_prob * 0.5 >= rand and is_there_pop_in_surrounding_cells:
                t1_pop = 1
            else:
                t1_pop = t0_pop * (1 + constant * t1_prob)
        if diff > 0.4:
            # proportioned change
            t1_pop = t0_pop * (1 + constant * t1_prob)

    if 0.09 < t1_prob <= 0.2:
        if diff < -0.4:
            # no change
            t1_pop = t0_pop
        if -0.4 <= diff < -0.15:
            # proportioned change
            t1_pop = t0_pop * (1 + constant * t1_prob)
        if -0.15 <= diff <= 0.15:
            # proportioned change or limited chance to pop birth in empty cell with neighbor
            if t0_pop == 0 and t1_prob * 0.5 >= rand and is_there_pop_in_surrounding_cells:
                t1_pop = 1
            else:
                t1_pop = t0_pop * (1 + constant * t1_prob)
        if 0.15 < diff <= 0.4:
            # proportioned change
            t1_pop = t0_pop * (1 + constant * t1_prob)
        if diff > 0.4:
            # no change
            t1_pop = t0_pop

    if 0.0 <= t1_prob <= 0.09:
        if diff < -0.4:
            # no change
            t1_pop = t0_pop
        if -0.4 <= diff < -0.15:
            # no change
            t1_pop = t0_pop
        if -0.15 <= diff <= 0.15:
            # proportioned change
            t1_pop = t0_pop * (1 + constant * t1_prob)
        if 0.15 < diff <= 0.4:
            # no change
            t1_pop = t0_pop
        if diff > 0.4:
            # no change
            t1_pop = t0_pop

    if -0.09 <= t1_prob < 0.0:
        if diff < -0.4:
            # no change
            t1_pop = t0_pop
        if -0.4 <= diff < -0.15:
            # no change
            t1_pop = t0_pop
        if -0.15 <= diff <= 0.15:
            # proportioned change
            t1_pop = t0_pop * (1 + t1_prob)
        if 0.15 < diff <= 0.4:
            # no change
            t1_pop = t0_pop
        if diff > 0.4:
            # no change
            t1_pop = t0_pop

    if -0.2 <= t1_prob < -0.09:
        if diff < -0.4:
            # no change
            t1_pop = t0_pop
        if -0.4 <= diff < -0.15:
            # proportioned change
            t1_pop = t0_pop * (1 + t1_prob)
        if -0.15 <= diff <= 0.15:
            # proportioned change
            t1_pop = t0_pop * (1 + t1_prob)
        if 0.15 < diff <= 0.4:
            # proportioned change
            t1_pop = t0_pop * (1 + t1_prob)
        if diff > 0.4:
            # no change
            t1_pop = t0_pop

    if t1_prob < -0.2:
        t1_pop = t0_pop * (1 + t1_prob)

    if t1_pop > 5.0:
        t1_pop = 5

    return int(math.ceil(t1_pop))


def historical_probabilities_and_population(t0_prob, t0_pop, t1_prob, t1_pop):
    # calculate a random float between 0 and 2
    rand = np.random.uniform(0, 2, 1)[0]
    diff = t1_prob - t0_prob

    if t1_prob > 0.35:
        if diff < -0.5:
            t1_pop = t0_pop
        if -0.5 <= diff < -0.25:
            t1_pop = int(t0_pop * (1 + t1_prob) + rand)
        if -0.25 <= diff <= 0.25:
            t1_pop = int(t0_pop * (1 + t1_prob) + rand) + 1
        if 0.25 < diff <= 0.5:
            t1_pop = int(t0_pop * (1 + t1_prob) + rand)
        if diff > 0.5:
            t1_pop = t0_pop

    if 0 < t1_prob <= 0.35:
        if diff < -0.5:
            t1_pop = t0_pop
        if -0.5 <= diff < -0.25:
            t1_pop = int(t0_pop * (1 + t1_prob))
        if -0.25 <= diff <= 0.25:
            t1_pop = int(t0_pop * (1 + t1_prob) + rand)
        if 0.25 < diff <= 0.5:
            t1_pop = int(t0_pop * (1 + t1_prob))
        if diff > 0.5:
            t1_pop = t0_pop

    if -0.35 <= t1_prob < 0:
        if diff < -0.5:
            t1_pop = t0_pop
        if -0.5 <= diff < -0.25:
            t1_pop = int(t0_pop * (1 + t1_prob))
        if -0.25 <= diff <= 0.25:
            t1_pop = int(t0_pop * (1 + t1_prob))
        if 0.25 < diff <= 0.5:
            t1_pop = int(t0_pop * (1 + t1_prob))
        if diff > 0.5:
            t1_pop = t0_pop

    if t1_prob < -0.35:
        if diff < -0.5:
            t1_pop = t0_pop
        if -0.5 <= diff < -0.25:
            t1_pop = int(t0_pop * (1 + t1_prob))
        if -0.25 <= diff <= 0.25 and t0_pop > 0:
            t1_pop = int(t0_pop * (1 + t1_prob) - rand)
        if 0.25 < diff <= 0.5:
            t1_pop = int(t0_pop * (1 + t1_prob))
        if diff > 0.5:
            t1_pop = t0_pop

    return t1_pop


def conditional_based_pop_growth(t0_pop, t1_prob, t1_pop):
    # if the probability is positive and large, grow population quickly (even in dead cells)
    if t1_prob > 0.5:
        t1_pop = int(t0_pop * (1 + t1_prob)) + 2
    # if the prob is positive and average, grow population slowly (even in dead cells)
    elif 0.25 <= t1_prob <= 0.5:
        t1_pop = int(t0_pop * (1 + t1_prob)) + 1
    # if the prob is positive and small OR prob is negative, change population in living cells only
    else:
        t1_pop = int(t0_pop * (1 + t1_prob))

    return t1_pop


def normal_pop_growth_with_random_conditions(t0_pop, t1_prob, t1_pop):
    # calculate a random float between -1 and 1
    rand = np.random.uniform(-1, 1, 1)
    # print(f"\nrand = {rand}")
    # print(f"prob = {data_t1[c][2]}")
    # run the conditionals for pop growth/decline
    if 0.0 < t1_prob <= rand:
        # new pop = (old pop + 1) * 10 * new prob
        t1_pop = 1 + int(t0_pop * (1 + t1_prob))
        # print(f"grew the pop from {data_t0[c][3]} to {data_t1[c][3]}")
    if rand < t1_prob < 0.0:
        # new pop = old pop * new prob
        t1_pop = int(t0_pop * (1 + t1_prob))
        # print(f"declined the pop from {data_t0[c][3]} to {data_t1[c][3]}")
    return t1_pop


def place_into_csv_file(array_of_points, save_folder_path, file_name):
    # place values into a cvs file
    gt_file = open(f"{save_folder_path}/{file_name[15:]}", "w")
    # save values to a file
    for v in array_of_points:
        np.savetxt(gt_file, v, delimiter=',')


# assign the directory for all the csv files
directory = "/Users/andrewphillips/Documents/Documents - Andrew’s MacBook Air/College/PaoloLab/uri_soft_wip/cyanobloom_simulation/ground_truths/SabattusPond/2016/population_data_at_each_cell_2016"
# save path for txt files containing ground truth coordinates
save_path = "/Users/andrewphillips/Documents/Documents - Andrew’s MacBook Air/College/PaoloLab/uri_soft_wip/cyanobloom_simulation/ground_truths/SabattusPond/2016/ground_truth_coordinates"
# initialize a list of filename strings
filename_list = []
# iterate through each file in the specified directory. append the filename to filename_list
for file in os.listdir(directory):
    filename_list.append(os.fsdecode(file))
# sort the list to be in alphabetical order
filename_list = sorted(filename_list)
# remove the first part
filename_list = filename_list[1:]

# initialize count
count = 0

while count <= len(filename_list) - 2:
    # read result file for current time step
    data_t0 = pd.read_csv(directory + "/" + filename_list[count], encoding="utf-8", index_col=0,
                          on_bad_lines='warn')
    # read result file for next time step
    data_t1 = pd.read_csv(directory + "/" + filename_list[count + 1], encoding="utf-8", index_col=0,
                          on_bad_lines='warn')
    # reset indexes and reset the data_t1 population back to zero
    data_t0.reset_index()
    data_t1.reset_index()
    data_t1["Population"] = 0

    # convert the pandas dataframes into numpy objects
    data_t0.to_numpy()
    data_t1.to_numpy()
    # make dataframes into arrays
    data_t0 = np.array(data_t0)
    data_t1 = np.array(data_t1)

    # initialize other_count
    c = 0
    # initialize the list of gaussian particles. this resets with each time step
    guass_distribution_list = []
    # run through each coordinate in time step
    while c < len(data_t1):
        # in case t0 and t1 arent the same size
        if c < len(data_t0):
            # data_t1[c][3] = normal_pop_growth_with_random_conditions(data_t0[c][3], data_t1[c][2], data_t1[c][3])
            # data_t1[c][3] = conditional_based_pop_growth(data_t0[c][3], data_t1[c][2], data_t1[c][3])
            # data_t1[c][3] = historical_probabilities_and_population(data_t0[c][2], data_t0[c][3],
            # data_t1[c][2], data_t1[c][3])

            data_t1[c][3] = historical_probs_and_pop_and_neighbor_pop(t0_prob=data_t0[c][2],
                                                                      t0_pop=data_t0[c][3],
                                                                      t1_prob=data_t1[c][2],
                                                                      t1_pop=data_t1[c][3],
                                                                      t0_surrounding_cells=data_t1[c - 3:c + 4])

        # use a 2d gaussian distribution to create particle distributions.
        # mean = coordinate
        # cov = a sigma of 15
        # num = the population calculated in the conditions above
        mean = [data_t1[c][0], data_t1[c][1]]
        cov_matrix = np.array([[225 / 111139 ** 2, 0],
                               [0, 225 / 111139 ** 2]])
        # check if there are any particles to create
        if data_t1[c][3] > 0.0:
            # append value to the list
            guass_distribution_list.append(np.random.multivariate_normal(mean, cov_matrix, int(data_t1[c][3])))
        if c == 1000:
            print(f"{count} / {len(filename_list) - 2}\n"
                  f"Date/time: {filename_list[count + 1]}\n"
                  f"Num of particles at {[data_t1[c][0], data_t1[c][1]]}: {int(data_t1[c][3])}")
        # add to the counter
        c += 1

    # convert back to a dataframe
    df_t1 = pd.DataFrame(data_t1, columns=['x', 'y', 'Hotspot Prob', 'Population'])
    # save the data_t1 data into a csv
    df_t1.to_csv(directory + "/" + filename_list[count + 1])

    # place ground truth coordinates into txt file
    place_into_csv_file(guass_distribution_list, save_path, filename_list[count + 1])

    count += 1
