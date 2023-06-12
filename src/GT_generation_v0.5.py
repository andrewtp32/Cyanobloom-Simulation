"""
This script obtains contours of a given lake/pond from an openstreetmap image.
Contours of "human zones" are obtained from the map image. Human zones are .
All logged points are added to a csv file that is placed in a folder that is in the desktop directory.

User controller variables and inputs:
time_steps - the number of evolutions that the simulation goes through. 10 is pretty standard for this.
image path - the original screenshot of the OpenStreetMap
size - number of ground truths to be generated per human zone (guassian distribution)
alpha - strength of the vortex (maybe use a slider for this)
ground truth evolution save path
water contour points save path
"""

import os
import colorsys
import cv2
import numpy as np
import tkinter as tk
import matplotlib.pyplot as plt
from numpy import genfromtxt
from tkinter import filedialog
from shapely.geometry import Polygon
from shapely.geometry import Point

# lake color is a type of blue. Define range of the blue water color in HSV:
# lower_range_water = np.array([90, 0, 181])
# upper_range_water = np.array([120, 189, 255])
# # "human zone" color is grey-ish. Define range of the grey human color in HSV:
# lower_range_human_zone = np.array([0, 0, 219])
# upper_range_human_zone = np.array([3, 3, 226])
# initialize row vectors for the lower and upper HSV ranges. a range for water zones and a range for human zones
lower_range_water = []
upper_range_water = []
lower_range_human_zone = []
upper_range_human_zone = []
# initialize variables for inputs
time_steps = 0
GTs_per_human_zone = 0
intensity_of_vortex = 0
# initialize strings for directory and file path
openstreetmap_file_path = ''
save_folder_path = ''
flight_path_file_path = ''


class BloomGUI:

    def __init__(self):
        # initialize the color picker image
        self.image = []

        # define the root window
        self.root = tk.Tk()
        # set title and geometry
        self.root.title("Cyanobacterial Bloom Spawning and Evolution Simulation")
        # self.root.geometry("500x300")

        # Set default font sizes
        title_font = ('Arial', 28)
        heading_font = ('Arial', 20)
        body_font = ('Arial', 16)

        # initialize variables for Entries
        self.var_time_steps = tk.IntVar(value=10)
        self.var_GTs_per_human_zone = tk.IntVar(value=500)
        self.var_intensity_of_vortex = tk.IntVar(value=40)

        # create labels for each input
        self.label_time_steps = tk.Label(self.root, text="Time steps:", font=body_font, padx=5,
                                         pady=10, anchor="e", justify="right")
        self.label_GTs_per_human_zone = tk.Label(self.root,
                                                 text="Ground truths generated\nper 'Human Zone':",
                                                 font=body_font, padx=5, pady=10, anchor="e",
                                                 justify="right")
        self.label_intensity_of_vortex = tk.Label(self.root, text="Intensity of lake vortex:",
                                                  font=body_font,
                                                  padx=5, pady=10, anchor="e", justify="right")
        self.label_instructions = tk.Label(self.root,
                                           text="Please input your desired values into each entry box below. The \n"
                                                "initialized values are merely recommendations. I recommend that \n"
                                                "you fiddle around with these values until you see something that \n"
                                                "you like.", font=heading_font, padx=5, pady=10,
                                           justify="left")

        # create entry widgets for each input
        self.entry_time_steps = tk.Entry(self.root, textvariable=self.var_time_steps, font=body_font)
        self.entry_GTs_per_human_zone = tk.Entry(self.root, textvariable=self.var_GTs_per_human_zone,
                                                 font=body_font)
        self.entry_intensity_of_vortex = tk.Entry(self.root, textvariable=self.var_intensity_of_vortex,
                                                  font=body_font)

        # create button widgets
        self.button_submit_entries = tk.Button(self.root, text="Submit",
                                               command=self.command_submit_entries)
        self.button_remove_entries = tk.Button(self.root, text="Remove ALL", command=self.command_remove_entries)
        self.button_choose_openstreetmap_file = tk.Button(self.root,
                                                          text="Choose OpenStreetMap\nscreenshot file\n(Click here)",
                                                          command=self.command_choose_openstreetmap_file, padx=5,
                                                          pady=10)
        self.button_choose_save_path = tk.Button(self.root,
                                                 text="Choose a folder to save\nground truth and water\nbody data (Click here)",
                                                 command=self.command_choose_save_path, padx=5, pady=10)
        self.button_choose_flight_path_save_file_path = tk.Button(self.root,
                                                                  text="Choose save path for drone\nflight path coordinates",
                                                                  command=self.command_choose_flight_path_save_file_path,
                                                                  padx=5,
                                                                  pady=10)
        self.button_pick_water_area_color = tk.Button(self.root,
                                                      text="Click on an area on the picture that you want to denote as 'water'",
                                                      command=self.command_pick_water_color,
                                                      padx=5,
                                                      pady=10)
        self.button_pick_human_area_color = tk.Button(self.root,
                                                      text="Click on an area on the picture that you want to denote as 'human zone'",
                                                      command=self.command_pick_human_color,
                                                      padx=5,
                                                      pady=10)

        # create text boxes
        self.textbox_save_directory_path = tk.Text(self.root, font=body_font, height=4, width=50)
        self.textbox_open_file_path = tk.Text(self.root, font=body_font, height=4, width=50)
        self.textbox_choose_flight_path_save_file_path = tk.Text(self.root, font=body_font, height=4, width=50)
        self.textbox_pick_water_area_color = tk.Text(self.root, font=body_font, height=2, width=50)
        self.textbox_pick_human_area_color = tk.Text(self.root, font=body_font, height=2, width=50)

        # Initialize text boxes
        self.textbox_save_directory_path.insert(tk.END, os.getcwd())
        self.textbox_open_file_path.insert(tk.END, os.getcwd())
        self.textbox_choose_flight_path_save_file_path.insert(tk.END, os.getcwd())

        # apply things to the grid. the grid is set up to be (10x3)
        # column 0
        self.label_instructions.grid(row=0, column=0, columnspan=3)
        self.label_time_steps.grid(row=1, column=0, sticky=tk.E)
        self.label_GTs_per_human_zone.grid(row=2, column=0, sticky=tk.E)
        self.label_intensity_of_vortex.grid(row=3, column=0, sticky=tk.E)
        self.button_choose_openstreetmap_file.grid(row=4, column=0, sticky=tk.E)
        self.button_pick_water_area_color.grid(row=5, column=0, sticky=tk.E)
        self.button_pick_human_area_color.grid(row=6, column=0, sticky=tk.E)
        self.button_choose_save_path.grid(row=7, column=0, sticky=tk.E)
        self.button_choose_flight_path_save_file_path.grid(row=8, column=0, sticky=tk.E)
        # column 1
        self.entry_time_steps.grid(row=1, column=1, columnspan=2, sticky=tk.W)
        self.entry_GTs_per_human_zone.grid(row=2, column=1, columnspan=2, sticky=tk.W)
        self.entry_intensity_of_vortex.grid(row=3, column=1, columnspan=2, sticky=tk.W)
        self.textbox_open_file_path.grid(row=4, column=1, columnspan=2, sticky=tk.W)
        self.textbox_pick_water_area_color.grid(row=5, column=1, columnspan=2, sticky=tk.W)
        self.textbox_pick_human_area_color.grid(row=6, column=1, columnspan=2, sticky=tk.W)
        self.textbox_save_directory_path.grid(row=7, column=1, columnspan=2, sticky=tk.W)
        self.textbox_choose_flight_path_save_file_path.grid(row=8, column=1, columnspan=2, sticky=tk.W)
        self.button_remove_entries.grid(row=9, column=1)
        # column 2
        self.button_submit_entries.grid(row=9, column=2, sticky=tk.W)

        # call the constructor
        self.root.mainloop()

    def color_picker_water(self, event, x, y, flags, param):
        global lower_range_water
        global upper_range_water
        if event == cv2.EVENT_LBUTTONDOWN:  # checks mouse left button down condition
            # Gives the BGR value. [::-1] flips the order of the values so it is RGB.
            rgb_color_at_cursor = self.image[y, x][::-1]
            # convert the normalized RGB color to HSV
            (h, s, v) = colorsys.rgb_to_hsv(rgb_color_at_cursor[0] / 255, rgb_color_at_cursor[1] / 255,
                                            rgb_color_at_cursor[2] / 255)
            # expand the hsv
            (h, s, v) = (int(h * 179), int(s * 255), int(v * 255))
            # subtract/add 5 to each hsv value for the lower and upper zones
            lower_range_water = np.array([int(0.95*h), int(0.95*s), int(0.95*v)])
            upper_range_water = np.array([int(1.05*h), int(1.05*s), int(1.05*v)])

    def color_picker_human(self, event, x, y, flags, param):
        global lower_range_human_zone
        global upper_range_human_zone
        if event == cv2.EVENT_LBUTTONDOWN:  # checks mouse left button down condition
            # Gives the BGR value. [::-1] flips the order of the values so it is RGB.
            rgb_color_at_cursor = self.image[y, x][::-1]
            # convert the normalized RGB color to HSV
            (h, s, v) = colorsys.rgb_to_hsv(rgb_color_at_cursor[0] / 255, rgb_color_at_cursor[1] / 255,
                                            rgb_color_at_cursor[2] / 255)
            # expand the hsv
            (h, s, v) = (int(h * 179), int(s * 255), int(v * 255))
            # subtract/add 5 to each hsv value for the lower and upper zones
            lower_range_human_zone = np.array([int(0.95*h), int(0.95*s), int(0.95*v)])
            upper_range_human_zone = np.array([int(1.05*h), int(1.05*s), int(1.05*v)])

    def command_submit_entries(self):
        global time_steps
        global GTs_per_human_zone
        global intensity_of_vortex
        # convert the entries into workable variables
        time_steps = self.var_time_steps.get()
        GTs_per_human_zone = self.var_GTs_per_human_zone.get()
        intensity_of_vortex = self.var_intensity_of_vortex.get()

        # close window
        self.root.destroy()

    def command_remove_entries(self):
        # reset the entries
        self.entry_time_steps.delete(0, tk.END)
        self.entry_GTs_per_human_zone.delete(0, tk.END)
        self.entry_intensity_of_vortex.delete(0, tk.END)
        self.textbox_open_file_path.delete('1.0', 'end')
        self.textbox_save_directory_path.delete('1.0', 'end')
        self.textbox_pick_water_area_color.delete('1.0', 'end')
        self.textbox_pick_human_area_color.delete('1.0', 'end')

    def command_choose_openstreetmap_file(self):
        global openstreetmap_file_path
        # get the current working directory
        current_directory = os.getcwd()
        # open the "find file" window
        openstreetmap_file_path = filedialog.askopenfile(parent=self.root, initialdir=current_directory,
                                                         title="Please select an image file.")
        # extract the "name" from the TextIOWrapper type
        openstreetmap_file_path = openstreetmap_file_path.name
        # reset the textbox to an oen slate
        self.textbox_open_file_path.delete('1.0', 'end')
        # extract name from openstreetmap_file_path
        self.textbox_open_file_path.insert(tk.END, openstreetmap_file_path)

    def command_choose_save_path(self):
        global save_folder_path
        # get the current working directory
        current_directory = os.getcwd()
        # open the "find directory" window
        save_folder_path = filedialog.askdirectory(parent=self.root, initialdir=current_directory,
                                                   title="Please select a save path")
        # reset the textbox to an oen slate
        self.textbox_save_directory_path.delete('1.0', 'end')
        # place the file path string into the textbox
        self.textbox_save_directory_path.insert(tk.END, save_folder_path)

    def command_choose_flight_path_save_file_path(self):
        global flight_path_file_path
        # get the current working directory
        current_directory = os.getcwd()
        # open the "find directory" window
        flight_path_file_path = filedialog.askdirectory(parent=self.root, initialdir=current_directory,
                                                        title="Please select a save path")
        # reset the textbox to an oen slate
        self.textbox_choose_flight_path_save_file_path.delete('1.0', 'end')
        # place the file path string into the textbox
        self.textbox_choose_flight_path_save_file_path.insert(tk.END, flight_path_file_path)

    def command_pick_water_color(self):
        global openstreetmap_file_path
        global lower_range_water
        global upper_range_water
        lower_range_water = []
        upper_range_water = []
        # title the image popup
        cv2.namedWindow("Color Picker")
        # callback for when the mouse clicks on an area of the image popup
        cv2.setMouseCallback("Color Picker", self.color_picker_water)
        # read the image and initialize the image variable
        self.image = cv2.imread(openstreetmap_file_path)
        # while loop will break when the empty color range vectors are filled
        while np.size(lower_range_water) < 1:
            # show image
            cv2.imshow("Color Picker", self.image)
            cv2.waitKey(1)
        # destroy the window
        cv2.destroyAllWindows()
        # clear the text box
        self.textbox_pick_water_area_color.delete('1.0', 'end')
        # place range info into text box
        self.textbox_pick_water_area_color.insert(tk.END, f"Water color lower boundary RBG: {lower_range_water}\n"
                                                          f"Water color upper boundary RBG: {upper_range_water}")

    def command_pick_human_color(self):
        global lower_range_human_zone
        global upper_range_human_zone
        global openstreetmap_file_path
        lower_range_human_zone = []
        upper_range_human_zone = []
        # title the image popup
        cv2.namedWindow("Color Picker")
        # callback for when the mouse clicks on an area of the image popup
        cv2.setMouseCallback("Color Picker", self.color_picker_human)
        # read the image and initialize the image variable
        self.image = cv2.imread(openstreetmap_file_path)
        # while loop will break when the empty color range vectors are filled
        while np.size(lower_range_human_zone) < 1:
            # show image
            cv2.imshow("Color Picker", self.image)
            cv2.waitKey(1)
        # destroy the window
        cv2.destroyAllWindows()
        # clear the text box
        self.textbox_pick_human_area_color.delete('1.0', 'end')
        # place range info into text box
        self.textbox_pick_human_area_color.insert(tk.END, f"Human zone lower boundary RBG: {lower_range_human_zone}\n"
                                                          f"Human zone upper boundary RBG: {upper_range_human_zone}")


def generate_drone_desired_positions_for_images_txt_file(x, y, width, height):
    # Create an mgrid for the values in the x and y directions with an interval of 20 units
    X, Y = np.mgrid[x:x + width:50, y:y + height:50]

    # np.ravel() will unravel all elements from a N-dimensional matrix into a 1D array zip() functions return a "zip"
    # object which is an iterator of tuples where the first item in each passed iterator is paired together,
    # and then the second items are pair, and so on. Basically, it takes two arrays, makes them vertical,
    # and slaps them together. list() converts the array zip object to a list np.vstack() simply stacks vectors on
    # top of each other (just as the way it sounds)
    drone_flight_path = np.vstack(list(zip(X.ravel(), Y.ravel())))
    # print the stacked list of
    print(f"There are {len(drone_flight_path)} points where the drone will take a picture\n")
    flight_path_file = open(f"{flight_path_file_path}/drone_flight_path.txt", "w")
    # save values to a file
    for v in drone_flight_path:
        np.savetxt(flight_path_file, [v], delimiter=',')


def rearrange_ground_truths_to_within_boundary(water_cnt, new_ground_truths):
    rearranged_ground_truths = []  # initialize the array
    water_cnt_poly = Polygon(water_cnt)  # create water contour polygon type
    for gt in new_ground_truths:
        ground_truth_point_type = Point(gt)
        # use "contains(ground_truth)" function to get boolean for if test_point is within the boundaries of polygon
        # print(f"The point {test_point} is within the polygon: {intersection_poly.contains(test_point)}")
        if water_cnt_poly.contains(ground_truth_point_type) == 0:
            # for loop through each water contour and search for the contour that is closest to the ground truth
            for cnt in water_cnt:
                shortest_distance = 99999999
                # find distance between contour point and ground truth
                distance = np.sqrt((gt[1] - cnt[1]) ** 2 + (gt[0] - cnt[0]) ** 2)
                # find the closest contour to the ground truth
                if distance < shortest_distance:
                    gt = cnt  # assign the closest contours position to the ground truth
        rearranged_ground_truths.append(gt)
    return rearranged_ground_truths


def find_center_point_of_water_body(water_cnt):
    contour_poly = Polygon(water_cnt)  # create a polygon out of the water contours
    center_point_water_body = contour_poly.centroid.coords  # centroid finds the polygon's center of gravity
    return list(center_point_water_body)[0]  # returns the "float" type of the center point


def vortex_potential_field(ground_truths, center_of_water_body):
    # strength of vortex
    alpha = intensity_of_vortex
    # initialize the new ground truths array
    new_ground_truths = []
    for gt in ground_truths:
        # calculating distance between ground truth and obstacle
        distance = np.sqrt((gt[1] - center_of_water_body[1]) ** 2 + (gt[0] - center_of_water_body[0]) ** 2)
        # compute the delta in the x and y directions as result of the vortex field
        if distance == 0:
            deltaX = 0
            deltaY = 0
        else:
            # (vortex equation) +/- 5% randomness
            deltaX = ((alpha * gt[1]) / (2 * 3.14 * distance ** 2)) + 0.01 * np.random.randint(low=-5, high=5)
            deltaY = (-(alpha * gt[0]) / (2 * 3.14 * distance ** 2)) + 0.01 * np.random.randint(low=-5, high=5)
        # apply the deltaX and deltaY to the ground truth (along with some noise)
        new_gt = [gt[0] + deltaX, gt[1] + deltaY]
        # append the new ground truth to the ground truth list
        new_ground_truths.append(new_gt)
    return new_ground_truths


def ground_truth_evolution_and_save_to_csv(water_cnt, ground_truths_array):
    # initialize the counter
    count = 0
    # find the center point of the water body
    center_point_water_body = find_center_point_of_water_body(water_cnt)
    # use the while loop to count through 10 time steps
    while count < time_steps:
        # call the vortex potential field function
        ground_truths_array = vortex_potential_field(ground_truths_array, center_point_water_body)
        # if a ground truth falls outside water contour, displace it to the closest water contour
        ground_truths_array = rearrange_ground_truths_to_within_boundary(water_cnt, ground_truths_array)
        # invert the points (water_contours, raw_guassian_distribution_array) in y direction
        invert_water_cnt, invert_ground_truths_array = invert_and_prepare_points_for_csv(water_cnt, ground_truths_array)
        # place the data into csv files
        place_into_csv_file(invert_ground_truths_array, "ground_truths", count)
        place_into_csv_file(invert_water_cnt, "water_contours", count)
        # add one to the count
        count += 1


def remove_points_outside_of_water_body(water_cnt, ground_truths_array):
    inspected_ground_truths = []
    water_cnt_poly = Polygon(water_cnt)
    for ground_truth in ground_truths_array:
        ground_truth_point_type = Point(ground_truth)
        # use "contains(ground_truth)" function to get boolean for if test_point is within the boundaries of polygon
        # print(f"The point {test_point} is within the polygon: {intersection_poly.contains(test_point)}")
        if water_cnt_poly.contains(ground_truth_point_type) == 1:
            # save the iteration of where the TRUE test point is within the test_point list
            inspected_ground_truths.append(ground_truth)
    return inspected_ground_truths


def invert_and_prepare_points_for_csv(a, b):
    all_points = []  # initialize list
    length_of_a = len(a)  # find the length of the first list
    length_of_b = len(b)  # find the length of the second list
    # use for loops to create "all_points" (aka - the combination of list a and list b)
    for value in a:
        all_points.append(value)
    for value in b:
        all_points.append(value)
    # invert the points so the lake isnt upside down
    all_points_inverted = invert_points_in_y_direction(all_points)
    # separate the "all_points" list back into "a" and "b"
    a_inverted = all_points_inverted[:(length_of_a - 1)]
    # slice the "all_points_inverted" list at the "length_of_a" iteration
    b_inverted = all_points_inverted[length_of_a:]

    return a_inverted, b_inverted


def generate_gaussian_distribution(center_pt, x_values, y_values):
    # for each center point, create a gaussian distribution with mean based upon the center point and the covariance
    # based upon the x and the y points of the sphere of influence
    mean = [center_pt[0], center_pt[1]]
    # create a covariance matrix by making an array from x_values and y_values and then using a numpy function.
    cov_matrix = np.cov(np.array([x_values, y_values]))
    # simple bi-variate normal distribution
    guassian_distribution = np.random.multivariate_normal(mean, cov_matrix, GTs_per_human_zone)

    return guassian_distribution


def circular_values_human_sphere_of_influence(center_point, bounding_box_x, bounding_box_y):
    circle_values_x = []
    circle_values_y = []
    # create a linsapce of 200 points. this will give us a circle with "200 vertexes"
    thetas = np.linspace(0, 2 * np.pi, 200)
    # radius is the distance between the center point and a vertex of the bounding box (aka "x" from the
    # "find_contours" function).
    radius = np.sqrt((center_point[0] - bounding_box_x) ** 2 + (center_point[1] - bounding_box_y) ** 2)
    # for each theta value, synthesize a x,y value using the parametric equation for a circle. add the center_point[
    # 0] and the center_point[1] to shift the center point to its true/relative position.
    for theta in thetas:
        circle_values_x.append((radius * np.cos(theta)) + center_point[0])
    for theta in thetas:
        circle_values_y.append((radius * np.sin(theta)) + center_point[1])
    return circle_values_x, circle_values_y


def invert_points_in_y_direction(array_of_points):
    # print(f"The {range_type} array is a {type(array_of_points)} and looks like: {array_of_points}")
    # print(f"Length of the {range_type} array is: {len(array_of_points)}")
    inverted_array_of_points = []
    max_y_val = -9999999999

    # find maximum y value
    for v in array_of_points:
        if v[1] > max_y_val:
            max_y_val = v[1]
    # invert polygon in the y direction
    for v in array_of_points:
        # "max_y_val - v[1]" inverts the polygon in y direction
        inverted_array_of_points.append([v[0], max_y_val - v[1]])

    return inverted_array_of_points


def find_center_point_of_bounding_rectangle(x, y, width, height):
    center_point = [int((x + x + width) / 2), int((y + y + height) / 2)]
    return center_point


def place_into_csv_file(array_of_points, range_type, count):
    # place values into a cvs file
    if range_type == "ground_truths":
        gt_file = open(
            f"{save_folder_path}/{range_type}_{count}.txt",
            "w")
        # save values to a file
        for v in array_of_points:
            np.savetxt(gt_file, [v], delimiter=',')
    if range_type == "water_contours":
        cnt_file = open(
            f"{save_folder_path}/{range_type}_{count}.txt",
            "w")
        # save values to a file
        for v in array_of_points:
            np.savetxt(cnt_file, [v], delimiter=',')


def get_water_contours_and_initial_bloom_ground_truths(lower_range, upper_range, range_type):
    # get image from file path
    img = cv2.imread(openstreetmap_file_path)
    # Initialize new images:
    imgWithDrawnContours = img.copy()
    imgWithBoundingRectangles = img.copy()

    # initialize a list of valid water area contour points
    extracted_water_area_contour_points_array = []
    # initialize a list of matrices to be the unfiltered ground truths
    raw_guassian_distribution_array_of_vectors = []
    raw_guassian_distribution_array = []

    # Converting RGB to HSV:
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Threshold the HSV image to get only colors in defined HSV range
    mask = cv2.inRange(hsv, lower_range, upper_range)
    # Circumscribe the perimeter of each shape
    # use cv2.CHAIN_APPROX_SIMPLE to remove the redundant values
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # for each individual contour in the array of contours:
    for cnt in contours:
        area = cv2.contourArea(cnt)  # calculate area of the given shape
        if area > 250:
            cv2.drawContours(imgWithDrawnContours, cnt, -1, (0, 0, 255), 3)  # draws contours onto imgResult
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            x, y, width, height = cv2.boundingRect(approx)
            cv2.rectangle(imgWithBoundingRectangles, (x, y), (x + width, y + height), (0, 0, 255), 2)
            # transforming data into a 2d array
            if range_type == "water_area":
                generate_drone_desired_positions_for_images_txt_file(x, y, width, height)
                for point in cnt:
                    # print(f"'point' in 'cnt': {point}")
                    extracted_water_area_contour_points_array.append(point[0])
                gt_file = open(
                    f"/Users/andrewphillips/Desktop/Sabattus_water_contour_points.txt", "w")
                # save values to a file
                for v in extracted_water_area_contour_points_array:
                    np.savetxt(gt_file, [v], delimiter=',')
            if range_type == "human_area":
                # find the center point of the human contour area.
                center_point = find_center_point_of_bounding_rectangle(x, y, width, height)
                # use center point and the "x" value to create a list of points that make a circle add the center
                # point ot the list.
                human_sphere_of_influence_x, human_sphere_of_influence_y = circular_values_human_sphere_of_influence(
                    center_point, x, y)
                # create a Gaussian distribution using the "center points" as a mean for each distribution
                raw_guassian_distribution_at_center_point = generate_gaussian_distribution(center_point,
                                                                                           human_sphere_of_influence_x,
                                                                                           human_sphere_of_influence_y)
                # add the raw_guassian_distribution_at_center_points to the array
                # extracted_human_area_center_points_array.append(center_point)
                raw_guassian_distribution_array_of_vectors.append(raw_guassian_distribution_at_center_point)

    # the list of points is currently a matrix full of 2d position vectors. I want to change it so that the list is
    # just an array of position vectors. it is much simpler to work with.
    for matrix in raw_guassian_distribution_array_of_vectors:
        for vector in matrix:
            raw_guassian_distribution_array.append(vector)

    return extracted_water_area_contour_points_array, raw_guassian_distribution_array


# run GUI to get the user's desired time steps, number of GTs per human zone, and vortex intensity
BloomGUI()

# get contours of the water areas and human areas within specified image
water_contours, _ = get_water_contours_and_initial_bloom_ground_truths(lower_range_water, upper_range_water,
                                                                       "water_area")
_, raw_ground_truths = get_water_contours_and_initial_bloom_ground_truths(lower_range_human_zone,
                                                                          upper_range_human_zone,
                                                                          "human_area")

# test for which Gaussian distribution points reside within the water body.
# add the TRUE points to the "inspected_ground_truths" list.
inspected_ground_truths_array = remove_points_outside_of_water_body(water_contours, raw_ground_truths)

# create ten time steps that evolve the ground truths using the artificial potential fields.
# ground truths that escape the boundary of the water body should be displaced to the neared water contour
# log the positioning of the ground truths. add each time step to a csv file
ground_truth_evolution_and_save_to_csv(water_contours, inspected_ground_truths_array)

# read the ten txt files generated from "ground_truth_evolution_and_save_to_csv"
counter = 0
while counter < time_steps:
    # use the file path from the user input GUI
    ground_truth_scatter = genfromtxt(f'{save_folder_path}/ground_truths_{counter}.txt', delimiter=',')
    # use the saved folder path from the user input GUI
    water_contours_scatter = genfromtxt(f'{save_folder_path}/water_contours_{counter}.txt',
                                        delimiter=',')
    # print
    print(f"Number of ground truths = {len(ground_truth_scatter)} in time step {counter}")
    # create independent x and y lists for each array
    ground_truths_x, ground_truths_y = zip(*ground_truth_scatter)
    water_contours_x, water_contours_y = zip(*water_contours_scatter)
    # create figure for each time step
    plt.figure()
    plt.title(f"Time step = {counter}")
    plt.scatter(water_contours_x, water_contours_y, color="blue", s=1.5)
    plt.scatter(ground_truths_x, ground_truths_y, color="green", s=15)
    plt.show()
    # add one to the counter
    counter += 1

# show different cv images
'''
cv2.imshow("Original Image", img)
cv2.imshow("HSV Image", hsv)
cv2.imshow("Mask Image", mask)
cv2.imshow("Original image with contours", imgWithDrawnContours)
cv2.imshow("Original image with bounding rectangles", imgWithBoundingRectangles)
cv2.waitKey()'''
