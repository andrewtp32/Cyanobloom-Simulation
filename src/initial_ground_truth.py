import numpy as np
import matplotlib.pyplot as plt

bloom_points_list = []
x = -150
y = -100
x_vals = 0
y_vals = 0

while x <= 150:
    while y <= 100:
        bloom_points_list.append([x, y])
        y += 10
    y = -300
    x += 10

file = open(f"/home/drew/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/initial_ground_truth.txt", "w")
for v in bloom_points_list:
    np.savetxt(file, [v], delimiter=' ')