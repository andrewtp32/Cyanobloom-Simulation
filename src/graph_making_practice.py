import numpy as np
import scipy
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

count = 0
while count < 10:
    data = np.genfromtxt(
        f"/home/drew/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/3.Final_results/NEWindReflect/4"
        f"/ground_truth_minus_measurements/ground_truth_minus_measurements_{count}.txt",
        delimiter=' ')
    data_transpose = np.transpose(data)  # transpose data so that each dimension of the array is [[X], [Y], [Val]]
    # print(data)

    # --------------- Scatter plot 2D --------------------
    # remove all 0.0's from data and place the 1's in one array and the 0's in another array
    data_positives = []
    data_negatives = []
    for d in data:
        if d[2] > 0:
            data_positives.append(d)
        if d[2] < 0:
            data_negatives.append(d)
    # transpose data so that each dimension of the array is [[X], [Y], [Val]]
    data_positives_transpose = np.transpose(data_positives)
    data_negatives_transpose = np.transpose(data_negatives)
    # creating figure
    fig0 = plt.figure()
    ax = plt.axes()
    # create scatter plot stuff
    # if len(data_positives) > 0:
    #     plt.scatter(data_positives_transpose[0], data_positives_transpose[1], color='b', marker='^', alpha=0.6, label='False Negative')
    plt.scatter(data_negatives_transpose[0], data_negatives_transpose[1], color='g', marker='x', alpha=1, label='False Positive')
    # title
    plt.title(f"Measurement Error (time step = {count})", fontweight='bold')
    plt.xlabel("X-Axis (meters)", fontweight='bold')
    plt.ylabel("Y-Axis (meters)", fontweight='bold')
    # add legend
    plt.legend()
    # setting background color
    ax.set_facecolor('0.95')
    # add grid
    ax.grid()
    # restrict plot size
    plt.xlim([-150, 150])
    plt.ylim([-100, 100])
    # plt.show()
    # save the figure
    plt.savefig(f"/home/drew/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/3.Final_results"
                f"/NEWindReflect/4/Error_Graphs/measurement_error_step{count}_folder3.png")
    count += 1


'''
# ---------- Creating scatter of raw data with no colorization ----------
# creating figure
fig1 = plt.figure()
ax = plt.axes(projection='3d')
# creating plot
ax.scatter3D(data_transpose[0], data_transpose[1], data_transpose[2], color='green')
plt.title("Simple scatter")
# title and axis labels
plt.title("Raw data with no color code")
ax.set_xlabel('X-axis', fontweight='bold')
ax.set_ylabel('Y-axis', fontweight='bold')
ax.set_zlabel('Z-axis', fontweight='bold')
# show plot
# plt.show()

# ---------- creating scatter of raw data with color coding ----------
# creating figure
fig2 = plt.figure()
ax = plt.axes(projection='3d')
# adding x,y gridlines
ax.grid(b=True, color='grey', nlinestyle='-.', linewidth=0.3, alpha=0.2)
# creating color map
my_cmap = plt.get_cmap('hsv')
# creating plot
scatter_raw_w_color = ax.scatter3D(data_transpose[0], data_transpose[1], data_transpose[2],
                                   alpha=0.8,
                                   c=data_transpose[2],
                                   cmap=my_cmap)
# title and axis labels
plt.title("Raw data with color code")
ax.set_xlabel('X-axis', fontweight='bold')
ax.set_ylabel('Y-axis', fontweight='bold')
ax.set_zlabel('Z-axis', fontweight='bold')
fig2.colorbar(scatter_raw_w_color)
# show plot
# plt.show()

# ---------- creating scatter of "removed 0.0" data with color coding ----------
# remove all 0.0's from data
data_no_zeros = []
for d in data:
    if d[2] > 1e-5 or d[2] < -1e-5:
        data_no_zeros.append(d)
# print(len(data_no_zeros))
data_no_zeros_transpose = np.transpose(data_no_zeros)  # transpose so the dimensions of the array are columns
# creating figure
fig3 = plt.figure()
ax = plt.axes(projection='3d')
# adding x,y gridlines
ax.grid(b=True, color='grey', nlinestyle='-.', linewidth=0.3, alpha=0.2)
# creating plot
scatter_no_zero_w_color = ax.scatter3D(data_no_zeros_transpose[0], data_no_zeros_transpose[1],
                                       data_no_zeros_transpose[2],
                                       alpha=0.8,
                                       c=data_no_zeros_transpose[2],
                                       cmap='viridis')
# title and axis labels
plt.title("non-zeros data with color code")
ax.set_xlabel('X-axis', fontweight='bold')
ax.set_ylabel('Y-axis', fontweight='bold')
ax.set_zlabel('Z-axis', fontweight='bold')
fig3.colorbar(scatter_no_zero_w_color)
# show plot
# plt.show()

# ---------- creating scatter of "removed 0.0" data with color coding ----------
# remove all 0.0's from data
data_no_zeros = []
for d in data:
    if d[2] > 1e-3 or d[2] < -1e-3:
        data_no_zeros.append(d)
# print(len(data_no_zeros))
data_no_zeros_transpose = np.transpose(data_no_zeros)  # transpose so the dimensions of the array are columns
# creating figure
fig4 = plt.figure()
ax = plt.axes(projection='3d')
# adding x,y gridlines
ax.grid(b=True, color='grey', nlinestyle='-.', linewidth=0.3, alpha=0.2)
# creating plot
ax.plot_trisurf(data_no_zeros_transpose[0], data_no_zeros_transpose[1], data_no_zeros_transpose[2],
                cmap='viridis')
# title and axis labels
plt.title("non-zeros data with color code")
ax.set_xlabel('X-axis', fontweight='bold')
ax.set_ylabel('Y-axis', fontweight='bold')
ax.set_zlabel('Z-axis', fontweight='bold')
fig4.colorbar(scatter_no_zero_w_color)
# show plot
plt.show()
'''