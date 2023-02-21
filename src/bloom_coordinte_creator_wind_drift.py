# This script is responsible for creating a list of 2 dimentional points. The two dimentional points represent positions
# of cyanoblooms within a simulated gazebo world. The simulated gazebo world contains a rectangular pond (dimensions of
# 300m x 200m). The idea is to create ten lists of points that corelate to the positions of cyanoblooms over a period of
# time. The velocity of the cyanoblooms will model real world bloom velocities under weather conditions (primarily wind)

import numpy as np
import matplotlib.pyplot as plt
import time

# 2 dimentional means for the gaussian distributions.
starting_means = [[-135,8], [-135,16], [-135,32], [-135, 40], [-135, 48],
                  [-135,56], [-135, 64], [-135,72], [-135,80], [-135,88]]
covariance = [[1, 0]
              [0, 1]]
# generate cyanobloom points using the means in the starting_means list
for mean in starting_means:
    starting_bloom_positions_list = np.random.multivariate_normal(starting_means, covariance, 30)