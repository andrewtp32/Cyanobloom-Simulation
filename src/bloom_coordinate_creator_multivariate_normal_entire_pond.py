import numpy as np
import matplotlib.pyplot as plt

mean_A = (0, 0)
cov_A = [[10000, 100], [100, 10000]]
cov_B = [[50, 10], [10, 50]]
cov_C = [[100, 10], [10, 100]]
count = 0

A = np.random.multivariate_normal(mean_A, cov_A, 50)
while count < 10:
    B = []
    C = [[0, 0]]
    x_vals = []
    y_vals = []

    for i in A:
        new_point = np.random.multivariate_normal(i, cov_B)
        new_point_list = np.random.multivariate_normal(new_point, cov_C, 50)
        C = np.concatenate((C, new_point_list), axis=0)

    file = open(f"/home/drew/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/bloom_coordinates_random_blob/bloom_coordinates_random_blob{count}.txt", "w")
    for v in C:
        np.savetxt(file, [v], delimiter=' ')
    for v in C:
        x_vals = np.append(x_vals, v[0])
        y_vals = np.append(y_vals, v[1])
    plt.scatter(x_vals, y_vals)
    plt.show()

    count += 1
