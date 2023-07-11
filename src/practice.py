import pandas as pd
import numpy as np
import time
import os
from shapely.geometry import Point
from shapely.geometry import Polygon
import matplotlib.pyplot as plt

np.set_printoptions(threshold=np.inf)

start_time = time.time()

coordinate = [0.5, 0.5]


def find_closest(arr_a, arr_b, point):
    # stack the two vectors in the shape of a (N x 2)
    array = np.column_stack((np.asarray(arr_a), np.asarray(arr_b)))
    # return the index of the closest array value to the coordinate
    return (np.mean(np.abs(array - point), axis=1)).argmin()


def generate_gaussian_distribution(mu, num_generated):
    # Cov has a sigma of 15. The dividend is the number of meters in one coordinate length (this basically allows us to
    # scale the distribution)
    cov_matrix = np.array([[225 / 111139 ** 2, 0],
                           [0, 225 / 111139 ** 2]])
    # return a simple bi-variate normal distribution
    return np.random.multivariate_normal(mu, cov_matrix, num_generated)


"""
array_1 = np.asarray(np.random.random(10))
array_2 = np.asarray(np.random.random(10))
arr = np.column_stack((array_1, array_2))
arr_t = np.transpose(arr)
print(arr)
print(arr_t)

closest_x_index = (np.abs(arr_t[0] - coordinate[0])).argmin()

print(arr[closest_x_index])
"""
"""
points = np.asarray([[0.99500407, 0.10159837],
                     [0.51189846, 0.78066669],
                     [0.51189846, 0.05822802],
                     [0.51189846, 0.78066669],
                     [0.51189846, 0.49736552],
                     [0.51189846, 0.64277285],
                     [0.84075243, 0.33712056],
                     [0.81007884, 0.19461181],
                     [0.57364846, 0.27067382],
                     [0.77065887, 0.73072798]])
points_t = np.transpose(points)

print(points)
print(points_t)

closest_x_index = (np.abs(points_t[0] - coordinate[0])).argmin()

print(points[closest_x_index][0])
"""
"""
points = np.asarray([[0.99500407, 0.10159837],
                     [0.51189846, 0.78066669],
                     [0.51189846, 0.05822802],
                     [0.51189846, 0.78066669],
                     [0.51189846, 0.49736552],
                     [0.51189846, 0.64277285],
                     [0.84075243, 0.33712056],
                     [0.81007884, 0.19461181],
                     [0.57364846, 0.27067382],
                     [0.77065887, 0.73072798]])
points_t = np.transpose(points)

difference = (np.abs(points - coordinate))

mean = np.mean(difference, axis=1)

closest_to_coordinate_index = mean.argmin()

print(f"arr:\n{points}")
print(f"transpose:\n{points_t}")
print(f"diff:\n{difference}")
print(f"mean:\n{mean}")
print(f"point closest to coordinate:\n{points[closest_to_coordinate_index]}")
"""
"""
points = np.asarray([[0.99500407, 0.10159837],
                     [0.51189846, 0.78066669],
                     [0.51189846, 0.05822802],
                     [0.51189846, 0.78066669],
                     [0.51189846, 0.49736552],
                     [0.51189846, 0.64277285],
                     [0.84075243, 0.33712056],
                     [0.81007884, 0.19461181],
                     [0.57364846, 0.27067382],
                     [0.77065887, 0.73072798]])

closest_to_coordinate_index = (np.mean(np.abs(points - coordinate), axis=1)).argmin()

print(f"arr:\n{points}")
print(f"point closest to coordinate:\n{points[closest_to_coordinate_index]}")
"""
"""
array_1 = np.asarray(np.random.random(1000000))
array_2 = np.asarray(np.random.random(1000000))
arr = np.column_stack((array_1, array_2))

index = (np.mean(np.abs(arr - coordinate), axis=1)).argmin()

print(f"arr:\n{arr}")
print(f"point closest to coordinate:\n{arr[index]}")
"""
"""
array_1 = np.asarray(np.random.random(10))
array_2 = np.asarray(np.random.random(10))
arr = np.column_stack((array_1, array_2))

closest_index = find_closest(array_1, array_2, coordinate)

print(arr[closest_index], "\n")
"""
"""
array_1 = np.asarray(np.random.random(2))
array_2 = np.asarray(np.random.random(2))
arr_a = np.column_stack((array_1, array_2))
arr_b = np.column_stack((array_1, array_2))

appended = np.append(arr_a, arr_b, axis=0)

print(arr_a, "\n")
print(appended, "\n")

"""
"""
base_directory = "/Users/andrewphillips/Documents/Documents - Andrew’s MacBook Air/College/PaoloLab/uri_soft_wip/cyanobloom_simulation/jess_sat_data"
# read the csv for the month corresponding to the weather data
full_df = pd.read_csv(f'{base_directory}/hotspots_monthyear.csv')
# locate relevant satellite data
month_df = full_df.loc[(full_df['Year'] == 2016) & (full_df['Month'] == 5)]
# pull out just the coordinates
month_df_coordinates = month_df[['x', 'y']].copy()

month_df_coordinates.to_numpy()
month_coordinates_arr = np.array(month_df_coordinates)


print(month_coordinates_arr)
"""
"""
array_1 = np.asarray(np.random.random(10))
array_2 = np.asarray(np.random.random(10))
arr = np.column_stack((array_1, array_2))

closest_index = find_closest(array_1, array_2, coordinate)

if np.mean(abs(coordinate - arr[closest_index])) > 0.01:
    # if outside of body, bring the point back to the edge of the body
    coordinate = arr[closest_index]

print(arr)
print(arr[closest_index], "\n")
print(coordinate)


print("\n--- %s seconds ---" % (time.time() - start_time))
"""
"""
array_1 = np.asarray(np.random.random(3))
array_2 = np.asarray(np.random.random(3))
arr = np.column_stack((array_1, array_2))
print(arr)

arr = np.delete(arr, 2, 0)
print(arr)
"""
"""
arr = generate_gaussian_distribution(coordinate, 5)
"""
"""
array_1 = np.asarray(np.random.random(5))
array_2 = np.asarray(np.random.random(5))
arr = np.column_stack((array_1, array_2))
print(arr)
b = np.arange(3)
print(b)
c = 3
b = np.append(b, c)
print(b)
new_arr = np.delete(arr, b, 0)
print(new_arr)
"""
"""
array_1 = np.asarray(np.random.random(5))
array_2 = np.asarray(np.random.random(5))
arr = np.column_stack((array_1, array_2))

x, y = zip(*arr)
x = np.asarray(x)
y = np.asarray(y)

a = np.add(-0.5, np.random.rand(len(array_1)))

print(x)
print(a)

x = np.add(x, a)
arr = np.column_stack((x, y))

print(arr)
"""
"""
print(np.random.rand())
"""
"""
# find the center point of jess's data
base_dir = (os.getcwd())[:-4]
# read the csv for the month corresponding to the weather data
full_df = pd.read_csv(f'{base_dir}/jess_sat_data/hotspots_monthyear.csv')
# locate relevant satellite data
month_df = full_df.loc[(full_df['Year'] == 2016) & (full_df['Month'] == 5)]
# find the max and min values for each column
max_values = month_df.max()
min_values = month_df.min()
print(max_values[0], max_values[1])
print(min_values[0], min_values[1])
center_of_mass = [np.mean([max_values[0], min_values[0]]), np.mean([max_values[1], min_values[1]])]
print(center_of_mass)
"""

p = 28 / 4
print(p)
