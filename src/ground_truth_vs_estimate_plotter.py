import numpy as np
import scipy
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

count = 0
# initialize set to have a lattice point structure
lattice_points = np.mgrid[-150:150:2, -100:100:2].reshape(2, -1).T


def bivariate_pdf(X, Y, uX, uY):
    # return the bivariate gaussian pdf
    pdf = np.exp((-1 / 2) * (((X - uX) ** 2) + ((Y - uY) ** 2)))
    return pdf


# while loop if for iterating through the 10 versions of ground truth and estimate files
while count < 10:
    # read ground truth txt file
    bloom_ground_truth_wind_drift = np.genfromtxt(
        f"/home/drew/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/1.groundtruth_blocations/bloom_coordinates_northeast_wind_drift_with_noise_and_reflection/bloom_coordinates_northeast_wind_drift_with_noise_and_reflection{count}.txt",
        delimiter=' ')
    # read estimate txt file
<<<<<<< HEAD
    bloom_estimate_locations = np.genfromtxt(f"/home/drew/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/3.Final_results/NEWindReflect/1/raw_camera_results/initial_ground_truth_mu_AllCamRes-{count + 1}.txt", delimiter=' ', skip_header=1, skip_footer=1)
=======
    bloom_estimate_locations = np.genfromtxt(f"/home/drew/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/3.Final_results/NEWindReflect/4/graphs/initial_ground_truth__mu_AllWIs01-{count + 1}.txt",
        delimiter=' ', skip_header=1, skip_footer=1)
>>>>>>> 1971ad2fe63d6cc1d409e30cdbdf32782654234a

    lat_count = 1
    lattice_points_with_pdf_difference_array = []
    print(f"Time step = {count+1}")
    for lattice in lattice_points:  # iterate through each lattice point
        # print(f"Time step:{count + 1}/10, lattice count:{lat_count}/{len(lattice_points)}")
        gt_pdf_sum = 0   # initialize the sum of gt PDFs
        for ground_truth in bloom_ground_truth_wind_drift:  # iterate each through each bloom ground truth
            gt_pdf = bivariate_pdf(ground_truth[0], ground_truth[1], lattice[0], lattice[1])  # calculate bivariate pdf of gt point with lattice as the mean
            gt_pdf_sum += gt_pdf  # sum the pdfs of all gt's with respect to the lattice point under spotlight
        est_pdf_sum = 0  # initialize the sum of estimate PDFs
        for estimate in bloom_estimate_locations:  # iterate each through each bloom estimate
            est_pdf = bivariate_pdf(estimate[0], estimate[1], lattice[0], lattice[1]) # calculate bivariate pdf of gt point with estimate as the mean
            est_pdf_sum += est_pdf  # sum the pdfs of all estimates with respect to the lattice point under spotlight

        if gt_pdf_sum > 0.1:
            gt_pdf_sum = 1
        else:
            gt_pdf_sum = 0
        if est_pdf_sum > 0.1:
            est_pdf_sum = 1
        else:
            est_pdf_sum = 0

        lattice_points_with_pdf_difference_array.append([lattice[0], lattice[1], gt_pdf_sum-est_pdf_sum])  # append the lattice point and asociated pdf sum to an array
        lat_count += 1

<<<<<<< HEAD
    file = open(f"/home/drew/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/3.Final_results/NEWindReflect/1/ground_truth_minus_estimates/ground_truth_minus_estimates_{count}.txt", "w")
=======
    file = open(f"/home/drew/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/3.Final_results/NEWindReflect/4/ground_truth_minus_estimates/ground_truth_minus_estimates_{count}.txt", "w")
>>>>>>> 1971ad2fe63d6cc1d409e30cdbdf32782654234a
    for v in lattice_points_with_pdf_difference_array:
        np.savetxt(file, [v], delimiter=' ')

    count += 1
