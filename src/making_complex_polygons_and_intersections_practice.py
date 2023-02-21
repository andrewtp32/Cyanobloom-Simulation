import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
from shapely.geometry import Point

p = [(1, 1), (2, 2), (4, 2), (3, 1)]
q = [(1.5, 2), (3, 5), (5, 4), (3.5, 1)]
x = np.linspace(2, 4, 5)
y = np.linspace(1, 2, 4)
count = 0
x_mesh, y_mesh = np.meshgrid(x, y)
human_sphere_of_influence_poly_array = []
intersection_poly_array = []
test_points_Point_type = []
test_points_int_type = []
test_points_within_polygon = []  # list to contain the XY values of the TRUE test points
iteration_test_points_within_polygon = []  # list to contain the iteration of the TRUE test points

# here we make a list of test points. notice how i have to create a "Point" type, and then append it to the list!!!!!
for X in x:
    for Y in y:
        test_points_int_type.append([X, Y])
        point = Point(X, Y)
        test_points_Point_type.append(point)

p_poly = Polygon(p)
q_poly = Polygon(q)
# print(p_poly.intersects(q_poly))  # True
# print(p_poly.intersection(q_poly).area)  # 1.0
intersection_poly = p_poly.intersection(q_poly)  # find vertexes of the intersection
# print(intersection_poly)

'''water_contours_poly = Polygon(water_contours)  # create polygon
for human_sphere_of_influence in human_sphere_of_influences_array:
    human_sphere_of_influence_poly = Polygon(human_sphere_of_influence)
    human_sphere_of_influence_poly_array.append(human_sphere_of_influence_poly)
'''

for test_point in test_points_Point_type:
    # use "intersection_poly.contains(test_point)" to get boolean for if test_point is within the boundaries of polygon
    # print(f"The point {test_point} is within the polygon: {intersection_poly.contains(test_point)}")
    if intersection_poly.contains(test_point) == 1:
        # save the iteration of where the TRUE test point is within the test_point list
        iteration_test_points_within_polygon.append(count)
    count += 1

print(f"True test points' iteration values: {iteration_test_points_within_polygon}")

# go through the test_points list and extract the points that match the iteration value
for iteration in iteration_test_points_within_polygon:
    # append the TRUE value to the list (by using its iteration value
    test_points_within_polygon.append(test_points_int_type[iteration])

print(f"The TRUE point values are: {test_points_within_polygon}")

p.append(p[0])  # repeat the first point to create a 'closed loop'
q.append(q[0])  # repeat the first point to create a 'closed loop'
xp, yp = zip(*p)  # create lists of x and y values
xq, yq = zip(*q)  # create lists of x and y values
x_intersection, y_intersection = intersection_poly.exterior.xy  # turns the Shapely POLYGON "type" into an x and y array

plt.figure()
plt.plot(xp, yp)
plt.plot(xq, yq)
plt.plot(x_intersection, y_intersection)
plt.scatter(x_mesh, y_mesh)
plt.show()
