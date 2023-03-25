"""
Ground Truth Evolution and GUI

This python script takes user inputs and outputs CSV files for research use. More specifically, a GUI is ran to
gather inputs from the user. Inputs range from save paths, load paths, and simulation options. The system outputs
arrays of points for individual time steps. The arrays are arranged in a CSV file.

The "arrays" are filled with coordinates of cyanobacteria. At the introduction of each time step, cyanobacteria have
the possibility to be subjected to four outcomes: to spawn, to die, to duplicate, or to merely exist. The probability
of each outcome occurring is based upon a set of probabilities.

The probability of each outcome occurring is mostly predicated upon cyanobacteria's relationship to water
temperature. Warm water has been found to be the leading cause for cyanobacterial blooming events. In other words,
many studies have concluded that there is a direct correlation between increasing water temperature and the frequency
of blooming events. Other studies have gone further and have proposed evidence for warm water temperatures to be the
causation for blooming events. Water temperature data is gathered from the internet and the code is based on:
https://open-meteo.com/en/docs/historical-weather-api

This script generates outcomes for ground truths with probabilities based upon the aforementioned studies:
Spawn - look at papers
Death - look at papers
Reproduce - look at papers
Do Nothing - look at papers

Wind and water currents have their own affect on cyanobacteria populations. With each time step, wind and water
currents catalyze the migrations of cyanobacteria. In this simulator, the affect of wind and water currents are
mathematically represented by vector fields. The vector fields are applied to ground truths at each time step.

Once all time steps have been completed, the script shows a plot of the lake contours and ground truths.
"""

import os
import cv2
import csv
import codecs
import colorsys
import urllib.request
import urllib.error
import sys
import numpy as np
import tkinter as tk
import skfuzzy as fuzz
import matplotlib.pyplot as plt
from statistics import mean
from numpy import genfromtxt
from bs4 import BeautifulSoup
from tkinter import filedialog
from skfuzzy import control as ctrl
from shapely.geometry import Point
from shapely.geometry import Polygon

# initialize row vectors for the lower and upper HSV ranges.
lower_range_water = []
upper_range_water = []
# initialize variables for inputs
time_steps = 0
GTs_per_human_zone = 0
intensity_of_vortex = 0
# initialize strings for directory and file path
OSM_data_file_path = ''
openstreetmap_file_path = ''
save_folder_path = ''
flight_path_file_path = ''
# Optional start and end dates
# If nothing is specified, the forecast is retrieved.
# If start date only is specified, a single historical or forecast day will be retrieved
# If both start and and end date are specified, a date range will be retrieved
start_date = ''
end_date = ''


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
        self.var_start_date = tk.StringVar(value='2022-05-01')
        self.var_end_date = tk.StringVar(value='2022-10-31')

        # create labels for each input
        self.label_start_date = tk.Label(self.root, text="Start date (YYYY-MM-DD):", font=body_font, padx=5, pady=10,
                                         anchor="e", justify="right")
        self.label_end_date = tk.Label(self.root, text="End date (YYYY-MM-DD):", font=body_font, padx=5, pady=10,
                                       anchor="e", justify="right")
        self.label_instructions = tk.Label(self.root,
                                           text="Please input your desired values into each entry box below. The \n"
                                                "initialized values are merely recommendations. I recommend that \n"
                                                "you fiddle around with these values until you see something that \n"
                                                "you like.", font=heading_font, padx=5, pady=10,
                                           justify="left")

        # create entry widgets for each input
        self.entry_start_date = tk.Entry(self.root, textvariable=self.var_start_date, font=body_font)
        self.entry_end_date = tk.Entry(self.root, textvariable=self.var_end_date,
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
                                                                  text="Choose save path for drone\nflight path coordinates\n(Click here)",
                                                                  command=self.command_choose_flight_path_save_file_path,
                                                                  padx=5,
                                                                  pady=10)
        self.button_pick_water_area_color = tk.Button(self.root,
                                                      text="Click on an area on the picture that\nyou want to denote as 'water'",
                                                      command=self.command_pick_water_color,
                                                      padx=5,
                                                      pady=10)
        self.button_choose_OSM_data_file = tk.Button(self.root,
                                                     text="Choose OpenStreetMap\ndata file\n(Click here)",
                                                     command=self.command_choose_OSM_data_file,
                                                     padx=5,
                                                     pady=10)

        # create text boxes
        self.textbox_save_directory_path = tk.Text(self.root, font=body_font, height=4, width=50)
        self.textbox_open_file_path = tk.Text(self.root, font=body_font, height=4, width=50)
        self.textbox_choose_flight_path_save_file_path = tk.Text(self.root, font=body_font, height=4, width=50)
        self.textbox_pick_water_area_color = tk.Text(self.root, font=body_font, height=2, width=50)
        self.textbox_choose_OSM_data_file = tk.Text(self.root, font=body_font, height=4, width=50)

        # Initialize text boxes
        self.textbox_save_directory_path.insert(tk.END, os.getcwd())
        self.textbox_open_file_path.insert(tk.END, os.getcwd())
        self.textbox_choose_flight_path_save_file_path.insert(tk.END, os.getcwd())
        self.textbox_choose_OSM_data_file.insert(tk.END, os.getcwd())

        # apply things to the grid. the grid is set up to be (10x3)
        # column 0
        self.label_instructions.grid(row=0, column=0, columnspan=3)
        self.label_start_date.grid(row=1, column=0, sticky=tk.E)
        self.label_end_date.grid(row=2, column=0, sticky=tk.E)
        self.button_choose_OSM_data_file.grid(row=3, column=0, sticky=tk.E)
        self.button_choose_openstreetmap_file.grid(row=4, column=0, sticky=tk.E)
        self.button_pick_water_area_color.grid(row=5, column=0, sticky=tk.E)
        self.button_choose_save_path.grid(row=6, column=0, sticky=tk.E)
        self.button_choose_flight_path_save_file_path.grid(row=7, column=0, sticky=tk.E)
        # column 1
        self.entry_start_date.grid(row=1, column=1, columnspan=2, sticky=tk.W)
        self.entry_end_date.grid(row=2, column=1, columnspan=2, sticky=tk.W)
        self.textbox_choose_OSM_data_file.grid(row=3, column=1, columnspan=2, sticky=tk.W)
        self.textbox_open_file_path.grid(row=4, column=1, columnspan=2, sticky=tk.W)
        self.textbox_pick_water_area_color.grid(row=5, column=1, columnspan=2, sticky=tk.W)
        self.textbox_save_directory_path.grid(row=6, column=1, columnspan=2, sticky=tk.W)
        self.textbox_choose_flight_path_save_file_path.grid(row=7, column=1, columnspan=2, sticky=tk.W)
        self.button_remove_entries.grid(row=8, column=1)
        # column 2
        self.button_submit_entries.grid(row=8, column=2, sticky=tk.W)

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
            lower_range_water = np.array([int(0.95 * h), int(0.95 * s), int(0.95 * v)])
            upper_range_water = np.array([int(1.05 * h), int(1.05 * s), int(1.05 * v)])

    def command_choose_OSM_data_file(self):
        global OSM_data_file_path
        # get the current working directory
        current_directory = os.getcwd()
        # open the "find file" window
        OSM_data_file_path = filedialog.askopenfile(parent=self.root, initialdir=current_directory,
                                                    title="Please select an OSM data file.")
        # extract the "name" from the TextIOWrapper type
        OSM_data_file_path = OSM_data_file_path.name
        # reset the textbox to an oen slate
        self.textbox_choose_OSM_data_file.delete('1.0', 'end')
        # extract name from openstreetmap_file_path
        self.textbox_choose_OSM_data_file.insert(tk.END, OSM_data_file_path)

    def command_submit_entries(self):
        global start_date
        global end_date
        # convert the entries into workable variables
        start_date = self.var_start_date.get()
        end_date = self.var_end_date.get()

        # close window
        self.root.destroy()

    def command_remove_entries(self):
        # reset the entries
        self.entry_start_date.delete(0, tk.END)
        self.entry_end_date.delete(0, tk.END)
        self.textbox_open_file_path.delete('1.0', 'end')
        self.textbox_save_directory_path.delete('1.0', 'end')
        self.textbox_pick_water_area_color.delete('1.0', 'end')
        self.textbox_choose_OSM_data_file.delete('1.0', 'end')
        self.textbox_choose_flight_path_save_file_path.delete('1.0', 'end')

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
                                                   title="Please select a save path for the ground truth coordinates.")
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
                                                        title="Please select a save path for flight plan.")
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


def get_grid_of_points(array_of_points, res):
    # create a shape out of the contour points
    poly = Polygon(array_of_points)
    # boundaries of the shape
    lat_min, lon_min, lat_max, lon_max = poly.bounds

    # build a grid of points given a spacing parameter (aka resolution)
    grid = []  # array for the unfiltered points
    filtered_grid = []  # array for th filtered points
    for lat in np.arange(lat_min, lat_max, res):
        for lon in np.arange(lon_min, lon_max, res):
            # append the lattice point into the array
            grid.append([round(lat, 4), round(lon, 4)])
            # create a "Point" to test if the coordinate lies within the shape
            grid_cell_Point_type = Point((round(lat, 4), round(lon, 4)))
            # verify if "Point" fall within shape. if TRUE, append coordinate to the list
            if poly.contains(grid_cell_Point_type) == 1:
                filtered_grid.append([round(lat, 4), round(lon, 4)])

    return filtered_grid


def scale_and_invert_shape(array_of_points):
    array_of_points = np.array(array_of_points)
    # find the centroid of the shape
    M = cv2.moments(array_of_points)
    centroid_x = float(M['m10'] / M['m00'])
    centroid_y = float(M['m01'] / M['m00'])
    # translate shape to origin
    poly_at_origin = array_of_points - [centroid_x, centroid_y]
    # scale the shape (eventually this would be a scale based upon the coordinates given by the OSM data file)
    scale = 1
    ploy_scaled_at_origin = poly_at_origin * scale
    # reflect shape over the x axis
    poly_scaled_reflected_over_x_axis_at_origin = []
    for point in ploy_scaled_at_origin:
        poly_scaled_reflected_over_x_axis_at_origin.append([point[0], point[1] * -1])

    return poly_scaled_reflected_over_x_axis_at_origin


def read_OSM_xml(file_path):
    # Read the data inside the XML and put it under the variable name "data"
    with open(file_path, 'r') as f:
        OSM_data = f.read()

    # Passing the stored data inside the beautifulsoup parser, storing the returned object
    OSM_data = BeautifulSoup(OSM_data, "xml")

    # Finding all instances of tag `unique`
    bounds = OSM_data.find('bounds')
    # convert to a string
    bounds = str(bounds)

    # constants for the naming conventions in the xml file
    max_lat_name = 'maxlat="'
    min_lat_name = 'minlat="'
    max_lon_name = 'maxlon="'
    min_lon_name = 'minlon="'

    # slice the "bounds" string to find the location of each code
    max_lat_name_index = bounds.find(max_lat_name)
    max_lat_reduced = bounds[max_lat_name_index + len(max_lat_name):]
    # print(f"max_lat_reduced = {max_lat_reduced}")
    min_lat_name_index = bounds.find(min_lat_name)
    min_lat_reduced = bounds[min_lat_name_index + len(min_lat_name):]
    # print(f"min_lat_reduced = {min_lat_reduced}")
    max_lon_name_index = bounds.find(max_lon_name)
    max_lon_reduced = bounds[max_lon_name_index + len(max_lon_name):]
    # print(f"max_lon_reduced = {max_lon_reduced}")
    min_lon_index = bounds.find(min_lon_name)
    min_lon_reduced = bounds[min_lon_index + len(min_lon_name):]
    # print(f"min_lon_reduced = {min_lon_reduced}")

    # slice the reduced strings to get the coordinate only
    max_lat = float(max_lat_reduced[:max_lat_reduced.find('.') + 5])
    min_lat = float(min_lat_reduced[:min_lat_reduced.find('.') + 5])
    max_lon = float(max_lon_reduced[:max_lon_reduced.find('.') + 5])
    min_lon = float(min_lon_reduced[:min_lon_reduced.find('.') + 5])

    # find the coordinates of the corners of the bounding box
    corner_top_left = [max_lat, min_lon]
    corner_top_right = [max_lat, max_lon]
    corner_bottom_left = [min_lat, min_lon]
    corner_bottom_right = [min_lat, max_lon]
    # find the center point of the bounding box. round to just 4 decimal points
    center_coordinate = [round(mean([max_lat, min_lat]), 4), round(mean([max_lon, min_lon]), 4)]

    return corner_top_left, corner_top_right, corner_bottom_left, corner_bottom_right, center_coordinate


def fuzzy_logic(tim, irr_6hrs, win_spd):
    """
    FIRST LEG OF FUZZY SYSTEM
    """

    # Antecedent/Consequent objects hold universe variables, their domains, and membership functions(ranges)
    #   Inputs:
    #   * Average Wind Speed (m/s) in the last hour - ranges [0, 100]
    #   * Irradiance Flux (J·cm**-1·[6 h]**-1) in the last six hours - ranges [0, 4000]
    #   * Time of Day (hour) - ranges [0, 23]
    #   Outputs:
    #   * Stability of water column - ranges [0, 3]
    #   * Buoyancy of algae - ranges [0, 2]
    avg_wind_speed = ctrl.Antecedent(np.arange(0, 50.5, 0.5), 'avg_wind_speed')
    irradiance_flux_6hr = ctrl.Antecedent(np.arange(0, 4001, 1), 'irradiance_flux_6hr')
    time_of_day = ctrl.Antecedent(np.arange(0, 2500, 100), 'time_of_day')
    stability = ctrl.Consequent(np.arange(0, 6, 1), 'stability')
    buoyancy = ctrl.Consequent(np.arange(0, 5, 1), 'buoyancy')

    # Generate fuzzy membership functions
    avg_wind_speed['low'] = fuzz.trapmf(avg_wind_speed.universe, [0, 0, 1.5, 2.5])  # [beginning, peak, peak, end]
    avg_wind_speed['moderate'] = fuzz.trimf(avg_wind_speed.universe, [1.5, 2.5, 3.5])
    avg_wind_speed['high'] = fuzz.trimf(avg_wind_speed.universe, [2.5, 3.5, 4.5])
    avg_wind_speed['very_high'] = fuzz.trapmf(avg_wind_speed.universe, [3.5, 4.5, 50, 50])

    irradiance_flux_6hr['low'] = fuzz.trapmf(irradiance_flux_6hr.universe, [0, 0, 900, 1100])
    irradiance_flux_6hr['moderate'] = fuzz.trapmf(irradiance_flux_6hr.universe, [900, 1100, 1600, 2100])
    irradiance_flux_6hr['high'] = fuzz.trapmf(irradiance_flux_6hr.universe, [1600, 2100, 4000, 4000])

    time_of_day['night'] = fuzz.trapmf(time_of_day.universe, [0, 0, 400, 600]) + fuzz.trimf(time_of_day.universe,
                                                                                            [2200, 2400, 2400])
    time_of_day['morning'] = fuzz.trapmf(time_of_day.universe, [400, 600, 800, 1000])
    time_of_day['early_afternoon'] = fuzz.trapmf(time_of_day.universe, [800, 1000, 1200, 1400])
    time_of_day['late_afternoon'] = fuzz.trapmf(time_of_day.universe, [1200, 1400, 1600, 1800])
    time_of_day['evening'] = fuzz.trapmf(time_of_day.universe, [1600, 1800, 2200, 2400])

    # the levels of stability are represented with either a 1, 2, 3, or 4
    stability['low'] = fuzz.trapmf(stability.universe, [0, 0, 1, 2])
    stability['moderate'] = fuzz.trimf(stability.universe, [1, 2, 3])
    stability['high'] = fuzz.trimf(stability.universe, [2, 3, 4])
    stability['very high'] = fuzz.trapmf(stability.universe, [3, 4, 5, 5])
    # the levels of buoyancy are represented with either a 1, 2, or 3
    buoyancy['low'] = fuzz.trapmf(buoyancy.universe, [0, 0, 1, 2])
    buoyancy['moderate'] = fuzz.trimf(buoyancy.universe, [1, 2, 3])
    buoyancy['high'] = fuzz.trapmf(buoyancy.universe, [2, 3, 4, 4])

    # View the membership functions
    # avg_wind_speed.view()
    # irradiance_flux_6hr.view()
    # time_of_day.view()
    # stability.view()
    # buoyancy.view()
    # plt.show()

    # Fuzzy Rules - define the fuzzy relationship between input and output variables.
    #   1. Rules for stability of the water column.
    #   2. Rules for buoyancy.

    # Table 1
    stability_rule1 = ctrl.Rule(avg_wind_speed['very_high'] & irradiance_flux_6hr['high'], stability['low'])
    stability_rule2 = ctrl.Rule(avg_wind_speed['very_high'] & irradiance_flux_6hr['moderate'], stability['low'])
    stability_rule3 = ctrl.Rule(avg_wind_speed['very_high'] & irradiance_flux_6hr['low'], stability['low'])
    stability_rule4 = ctrl.Rule(avg_wind_speed['high'] & irradiance_flux_6hr['high'], stability['moderate'])
    stability_rule5 = ctrl.Rule(avg_wind_speed['high'] & irradiance_flux_6hr['moderate'], stability['low'])
    stability_rule6 = ctrl.Rule(avg_wind_speed['high'] & irradiance_flux_6hr['low'], stability['low'])
    stability_rule7 = ctrl.Rule(avg_wind_speed['moderate'] & irradiance_flux_6hr['high'], stability['high'])
    stability_rule8 = ctrl.Rule(avg_wind_speed['moderate'] & irradiance_flux_6hr['moderate'], stability['moderate'])
    stability_rule9 = ctrl.Rule(avg_wind_speed['moderate'] & irradiance_flux_6hr['low'], stability['low'])
    stability_rule10 = ctrl.Rule(avg_wind_speed['low'] & irradiance_flux_6hr['high'], stability['very high'])
    stability_rule11 = ctrl.Rule(avg_wind_speed['low'] & irradiance_flux_6hr['moderate'], stability['very high'])
    stability_rule12 = ctrl.Rule(avg_wind_speed['low'] & irradiance_flux_6hr['low'], stability['high'])
    # Table 2
    buoyancy_rule1 = ctrl.Rule(irradiance_flux_6hr['high'] & time_of_day['night'], buoyancy['high'])
    buoyancy_rule2 = ctrl.Rule(irradiance_flux_6hr['high'] & time_of_day['morning'], buoyancy['high'])
    buoyancy_rule3 = ctrl.Rule(irradiance_flux_6hr['high'] & time_of_day['early_afternoon'], buoyancy['moderate'])
    buoyancy_rule4 = ctrl.Rule(irradiance_flux_6hr['high'] & time_of_day['late_afternoon'], buoyancy['low'])
    buoyancy_rule5 = ctrl.Rule(irradiance_flux_6hr['high'] & time_of_day['evening'], buoyancy['moderate'])
    buoyancy_rule6 = ctrl.Rule(irradiance_flux_6hr['moderate'] & time_of_day['night'], buoyancy['high'])
    buoyancy_rule7 = ctrl.Rule(irradiance_flux_6hr['moderate'] & time_of_day['morning'], buoyancy['high'])
    buoyancy_rule8 = ctrl.Rule(irradiance_flux_6hr['moderate'] & time_of_day['early_afternoon'], buoyancy['moderate'])
    buoyancy_rule9 = ctrl.Rule(irradiance_flux_6hr['moderate'] & time_of_day['late_afternoon'], buoyancy['moderate'])
    buoyancy_rule10 = ctrl.Rule(irradiance_flux_6hr['moderate'] & time_of_day['evening'], buoyancy['moderate'])
    buoyancy_rule11 = ctrl.Rule(irradiance_flux_6hr['low'] & time_of_day['night'], buoyancy['high'])
    buoyancy_rule12 = ctrl.Rule(irradiance_flux_6hr['low'] & time_of_day['morning'], buoyancy['high'])
    buoyancy_rule13 = ctrl.Rule(irradiance_flux_6hr['low'] & time_of_day['early_afternoon'], buoyancy['moderate'])
    buoyancy_rule14 = ctrl.Rule(irradiance_flux_6hr['low'] & time_of_day['late_afternoon'], buoyancy['moderate'])
    buoyancy_rule15 = ctrl.Rule(irradiance_flux_6hr['low'] & time_of_day['evening'], buoyancy['high'])

    # View Rules
    # stability_rule1.view()
    # buoyancy_rule1.view()
    # plt.show()

    # Control System Creation and Simulation.
    # Now that we have our rules defined, we can simply create control systems via:
    stability_ctrl = ctrl.ControlSystem([stability_rule1, stability_rule2, stability_rule3, stability_rule4,
                                         stability_rule5, stability_rule6, stability_rule7, stability_rule8,
                                         stability_rule9, stability_rule10, stability_rule11, stability_rule12])
    buoyancy_ctrl = ctrl.ControlSystem([buoyancy_rule1, buoyancy_rule2, buoyancy_rule3, buoyancy_rule4,
                                        buoyancy_rule5, buoyancy_rule6, buoyancy_rule7, buoyancy_rule8,
                                        buoyancy_rule9, buoyancy_rule10, buoyancy_rule11, buoyancy_rule12,
                                        buoyancy_rule13, buoyancy_rule14, buoyancy_rule15])

    # In order to simulate this control system, we will create a ControlSystemSimulation. Think of this object
    # representing our controller applied to a specific set of circumstances.
    stability_state = ctrl.ControlSystemSimulation(stability_ctrl)
    buoyancy_state = ctrl.ControlSystemSimulation(buoyancy_ctrl)

    # We can now simulate our control system by simply specifying the inputs and calling the `compute` method.
    # Pass inputs to the ControlSystem using Antecedent labels with Pythonic API
    # Note: if you like passing many inputs all at once, use .inputs(dict_of_data)
    stability_state.input['avg_wind_speed'] = win_spd
    stability_state.input['irradiance_flux_6hr'] = irr_6hrs

    buoyancy_state.input['irradiance_flux_6hr'] = irr_6hrs
    buoyancy_state.input['time_of_day'] = tim

    # Crunch the numbers
    # These values are the outputs of the first leg of the fuzzy systems.
    # These values will be used as inputs for the next leg of the fuzzy system.
    stability_state.compute()
    buoyancy_state.compute()

    # Visualize
    # print(stability_state.output['stability'])
    # stability.view(sim=stability_state)
    # print(buoyancy_state.output['buoyancy'])
    # buoyancy.view(sim=buoyancy_state)
    # plt.show()

    '''
    SECOND LEG OF FUZZY SYSTEM
    '''

    # Antecedent/Consequent objects hold universe variables, their domains, and membership functions(ranges)
    #   Inputs:
    #   * Stability of water column - ranges [1, 4]
    #   * Buoyancy of algae - ranges [1, 3]
    #   Outputs:
    #   * Degree of Bloom Formation (crisp output, the probability of blooms appearing) - ranges [0, 100]
    #   * Degree of Bloom Disappearance (crisp output, the probability of blooms disappearing) - ranges [0, 100]
    stability_leg2 = ctrl.Antecedent(np.arange(0, 6, 1), 'stability_leg2')
    buoyancy_leg2 = ctrl.Antecedent(np.arange(0, 5, 1), 'buoyancy_leg2')
    degree_bloom_appearance = ctrl.Consequent(np.arange(0, 101, 1), 'degree_bloom_appearance')
    degree_bloom_disappearance = ctrl.Consequent(np.arange(0, 101, 1), 'degree_bloom_disappearance')

    # Generate fuzzy membership functions
    stability_leg2['low'] = fuzz.trimf(stability_leg2.universe, [0, 1, 2])
    stability_leg2['moderate'] = fuzz.trimf(stability_leg2.universe, [1, 2, 3])
    stability_leg2['high'] = fuzz.trimf(stability_leg2.universe, [2, 3, 4])
    stability_leg2['very high'] = fuzz.trimf(stability_leg2.universe, [3, 4, 5])

    buoyancy_leg2['low'] = fuzz.trimf(buoyancy_leg2.universe, [0, 1, 2])
    buoyancy_leg2['moderate'] = fuzz.trimf(buoyancy_leg2.universe, [1, 2, 3])
    buoyancy_leg2['high'] = fuzz.trimf(buoyancy_leg2.universe, [2, 3, 4])

    degree_bloom_appearance['very_low'] = fuzz.trapmf(degree_bloom_appearance.universe, [0, 0, 1, 9])
    degree_bloom_appearance['low'] = fuzz.trimf(degree_bloom_appearance.universe, [1, 9, 20])
    degree_bloom_appearance['moderate'] = fuzz.trimf(degree_bloom_appearance.universe, [9, 20, 45])
    degree_bloom_appearance['high'] = fuzz.trimf(degree_bloom_appearance.universe, [20, 45, 75])
    degree_bloom_appearance['very_high'] = fuzz.trapmf(degree_bloom_appearance.universe, [45, 75, 100, 100])

    degree_bloom_disappearance['very_low'] = fuzz.trapmf(degree_bloom_disappearance.universe, [0, 0, 1, 9])
    degree_bloom_disappearance['low'] = fuzz.trimf(degree_bloom_disappearance.universe, [1, 9, 20])
    degree_bloom_disappearance['moderate'] = fuzz.trimf(degree_bloom_disappearance.universe, [9, 20, 45])
    degree_bloom_disappearance['high'] = fuzz.trimf(degree_bloom_disappearance.universe, [20, 45, 75])
    degree_bloom_disappearance['very_high'] = fuzz.trapmf(degree_bloom_disappearance.universe, [45, 75, 100, 100])

    # View the membership functions
    # stability.view()
    # buoyancy.view()
    # degree_bloom_appearance.view()
    # degree_bloom_disappearance.view()

    # Fuzzy Rules - define the fuzzy relationship between input and output variables.
    #   3. Rules for degree of bloom appearance
    #   4. Rules for degree of bloom disappearance

    # Table 3
    degree_bloom_appearance_rule1 = ctrl.Rule(stability_leg2['low'] & buoyancy_leg2['high'],
                                              degree_bloom_appearance['low'])
    degree_bloom_appearance_rule2 = ctrl.Rule(stability_leg2['low'] & buoyancy_leg2['moderate'],
                                              degree_bloom_appearance['low'])
    degree_bloom_appearance_rule3 = ctrl.Rule(stability_leg2['low'] & buoyancy_leg2['low'],
                                              degree_bloom_appearance['very_low'])
    degree_bloom_appearance_rule4 = ctrl.Rule(stability_leg2['moderate'] & buoyancy_leg2['high'],
                                              degree_bloom_appearance['moderate'])
    degree_bloom_appearance_rule5 = ctrl.Rule(stability_leg2['moderate'] & buoyancy_leg2['moderate'],
                                              degree_bloom_appearance['low'])
    degree_bloom_appearance_rule6 = ctrl.Rule(stability_leg2['moderate'] & buoyancy_leg2['low'],
                                              degree_bloom_appearance['low'])
    degree_bloom_appearance_rule7 = ctrl.Rule(stability_leg2['high'] & buoyancy_leg2['high'],
                                              degree_bloom_appearance['very_high'])
    degree_bloom_appearance_rule8 = ctrl.Rule(stability_leg2['high'] & buoyancy_leg2['moderate'],
                                              degree_bloom_appearance['high'])
    degree_bloom_appearance_rule9 = ctrl.Rule(stability_leg2['high'] & buoyancy_leg2['low'],
                                              degree_bloom_appearance['moderate'])
    degree_bloom_appearance_rule10 = ctrl.Rule(stability_leg2['very high'] & buoyancy_leg2['high'],
                                               degree_bloom_appearance['very_high'])
    degree_bloom_appearance_rule11 = ctrl.Rule(stability_leg2['very high'] & buoyancy_leg2['moderate'],
                                               degree_bloom_appearance['high'])
    degree_bloom_appearance_rule12 = ctrl.Rule(stability_leg2['very high'] & buoyancy_leg2['low'],
                                               degree_bloom_appearance['moderate'])
    # Table 4
    degree_bloom_disappearance_rule1 = ctrl.Rule(stability_leg2['low'], degree_bloom_disappearance['very_high'])
    degree_bloom_disappearance_rule2 = ctrl.Rule(stability_leg2['moderate'], degree_bloom_disappearance['high'])
    degree_bloom_disappearance_rule3 = ctrl.Rule(stability_leg2['high'], degree_bloom_disappearance['low'])
    degree_bloom_disappearance_rule4 = ctrl.Rule(stability_leg2['very high'], degree_bloom_disappearance['very_low'])

    # Control System Creation and Simulation.
    # Now that we have our rules defined, we can simply create control systems via:
    degree_bloom_appearance_ctrl = ctrl.ControlSystem([degree_bloom_appearance_rule1, degree_bloom_appearance_rule2,
                                                       degree_bloom_appearance_rule3, degree_bloom_appearance_rule4,
                                                       degree_bloom_appearance_rule5, degree_bloom_appearance_rule6,
                                                       degree_bloom_appearance_rule7, degree_bloom_appearance_rule8,
                                                       degree_bloom_appearance_rule9, degree_bloom_appearance_rule10,
                                                       degree_bloom_appearance_rule11, degree_bloom_appearance_rule12])
    degree_bloom_disappearance_ctrl = ctrl.ControlSystem([degree_bloom_disappearance_rule1,
                                                          degree_bloom_disappearance_rule2,
                                                          degree_bloom_disappearance_rule3,
                                                          degree_bloom_disappearance_rule4])

    # In order to simulate this control system, we will create a ControlSystemSimulation. Think of this object
    # representing our controller applied to a specific set of circumstances.
    degree_bloom_appearance_state = ctrl.ControlSystemSimulation(degree_bloom_appearance_ctrl)
    degree_bloom_disappearance_state = ctrl.ControlSystemSimulation(degree_bloom_disappearance_ctrl)

    # We can now simulate our control system by simply specifying the inputs and calling the `compute` method.
    # Pass inputs to the ControlSystem using Antecedent labels with Pythonic API
    # Note: if you like passing many inputs all at once, use .inputs(dict_of_data)
    degree_bloom_appearance_state.input['stability_leg2'] = stability_state.output['stability']  # <- from leg one
    degree_bloom_appearance_state.input['buoyancy_leg2'] = buoyancy_state.output['buoyancy']  # <- from leg one

    degree_bloom_disappearance_state.input['stability_leg2'] = stability_state.output['stability']  # <- from leg one

    # Crunch the numbers
    # These values are the outputs of the second (and final) leg of the fuzzy systems.
    degree_bloom_appearance_state.compute()
    degree_bloom_disappearance_state.compute()

    # Visualize
    # print(f"Prob on bloom appearance: {degree_bloom_appearance_state.output['degree_bloom_appearance']\nProb bloom going away: {degree_bloom_disappearance_state.output['degree_bloom_disappearance']}")
    # degree_bloom_appearance.view(sim=degree_bloom_appearance_state)
    # degree_bloom_disappearance.view(sim=degree_bloom_disappearance_state)
    # plt.show()

    return degree_bloom_appearance_state.output['degree_bloom_appearance'], degree_bloom_disappearance_state.output[
        'degree_bloom_disappearance']


def create_CSV_from_API(coordinate, s_date, e_date):
    # fill the api query
    api_query = f"https://archive-api.open-meteo.com/v1/archive?latitude={coordinate[0]}&longitude={coordinate[1]}&start_date={s_date}&end_date={e_date}&hourly=direct_normal_irradiance,windspeed_10m,winddirection_10m&timezone=auto&windspeed_unit=ms&format=csv "
    # This part is for opening url and checking errors. not really sure what it all does exactly but kinda of get it
    try:
        CSV_bytes = urllib.request.urlopen(api_query)
    except urllib.error.HTTPError as e:
        error_info = e.read().decode()
        print('Error code: ', e.code, error_info)
        sys.exit()
    except urllib.error.URLError as e:
        error_info = e.read().decode()
        print('Error code: ', e.code, error_info)
        sys.exit()
    # Parse the results as CSV
    CSV = csv.reader(codecs.iterdecode(CSV_bytes, 'utf-8'))

    return CSV


def create_data_arr_from_CSV(CSV_file):
    # The first four rows contain the headers and the additional rows each contain the climate metrics for a single day
    # We will ignore the first four rows, then append all concurrent rows into an array

    # initialize data arr
    data_arr = []

    # for row in csv file
    for index_row, row in enumerate(CSV_file):
        # skip the headers
        if index_row > 3:
            # edit the first column; separate the date from time

            # find the Time separator
            T_index = row[0].find('T')
            # mark the "date"
            date = row[0][:T_index]
            # mark the "time"
            time_of_day = row[0][T_index + 1:].replace(":", "")
            # get irradiation
            irradiation = row[1]
            # get wind speed
            wind_speed = row[2]
            # get wind direction
            wind_direction = row[3]
            # append values to the list
            data_arr.append([date, time_of_day, irradiation, wind_speed, wind_direction])

    # If there are no CSV rows then something fundamental went wrong
    if len(data_arr) == 0:
        print('Sorry, but it appears that there was an error connecting to the weather server.')
        print('Please check your network connection and try again..')
    # If there is only one CSV row then we likely got an error from the server
    if len(data_arr) == 1:
        print('Sorry, but it appears that there was an error retrieving the weather data.')
        print('Error: ', CSV_file)

    return data_arr


def generate_drone_desired_positions_for_images_txt_file(array_of_points):
    # create a shape out of the contour points
    poly = Polygon(array_of_points)
    # boundaries of the shape
    lat_min, lon_min, lat_max, lon_max = poly.bounds
    # Create an mgrid for the values in the x and y directions with an interval of 20 units
    X, Y = np.mgrid[lon_min:lon_max:50, lat_min:lat_max:50]

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


def get_water_contours(lower_range, upper_range):
    # get image from file path
    img = cv2.imread(openstreetmap_file_path)

    # initialize a list of valid water area contour points
    extracted_water_area_contour_points_array = []
    # initialize variable to hold the largest water contour area
    largest_area = 0
    # initialize the array for the water contour with the largest area
    main_water_contour = []

    # Converting RGB to HSV:
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Threshold the HSV image to get only colors in defined HSV range
    mask = cv2.inRange(hsv, lower_range, upper_range)
    # Circumscribe the perimeter of each shape
    # use cv2.CHAIN_APPROX_SIMPLE to remove the redundant values
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # find the largest water contour in the list
    for cnt in contours:
        # calculate area of the given shape
        cnt_area = cv2.contourArea(cnt)
        if cnt_area > largest_area:
            main_water_contour = cnt

    # transforming data into a 2d array
    for point in main_water_contour:
        # print(f"'point' in 'cnt': {point}")
        extracted_water_area_contour_points_array.append(point[0])

    return extracted_water_area_contour_points_array


# run GUI
# get load path for screenshot of water, get the location of the lake, get save path for ground truths and flight path
BloomGUI()

# read the OSM xml file
coord_top_left, coord_top_right, coord_bottom_left, coord_bottom_right, coord_center = read_OSM_xml(OSM_data_file_path)

# get contours for the water area, get the coordinates for the bounding box
water_contour = get_water_contours(lower_range=lower_range_water,
                                   upper_range=upper_range_water)

# invert the contour points in the y direction
water_contour_reformatted = scale_and_invert_shape(water_contour)

# resolution will be 30 because we want each grid point to be 30 meters. apply 30 after we do the coordinate scaling
resolution = 10
# get a grid of points that fall within the contour shape
lattice_points_array = get_grid_of_points(water_contour_reformatted, resolution)

# create drone flight plan
generate_drone_desired_positions_for_images_txt_file(water_contour_reformatted)

# run API to get weather info for the lake location, convert data into workable arrays
# get API query and convert to CSV
CSV_text = create_CSV_from_API(coord_center, start_date, end_date)
# create an (n x 5) array. the columns are: [date, time, irradiation, wind speed (m/s), wind direction (degrees)]
data_array = create_data_arr_from_CSV(CSV_text)

# plot results
water_contour_reformatted_x, water_contour_reformatted_y = zip(*water_contour_reformatted)
lattice_points_array_x, lattice_points_array_y = zip(*lattice_points_array)

plt.figure()
plt.title(f"water contour map with lattice")
plt.scatter(water_contour_reformatted_x, water_contour_reformatted_y, color="blue", s=1.5)
plt.scatter(lattice_points_array_x, lattice_points_array_y, color="black", s=1.5)

plt.show()

# initialize list for irradiances
irradiance_list = []
count = 0
# For each time step:
for data_vector in data_array:
    # create easy-to-read variable names for the elements in the data vector
    date = data_vector[0]
    time = float(data_vector[1])
    irradiance = float(data_vector[2])
    wind_speed = float(data_vector[3])
    wind_direction = float(data_vector[4])
    # append irradiance to the list
    irradiance_list.append(irradiance)
    # calculate the irradiance sum over the last 6 hours
    irradiance_over_last_6hrs = sum(irradiance_list[-6:])
    # calculate fuzzy logic for blooms to spawn, reproduce, die, or do nothing based on temp data
    degree_bloom_appearance, degree_bloom_disappearance = fuzzy_logic(time, irradiance_over_last_6hrs, wind_speed)

    # use fuzzy logic for the actual action of spawn, death, reproduce, exist
    # use wind and vortex vectors to migrate the particles
    # move particles back to within the water boundary if they move outside
    # save array of points to a csv file

    if count % 168 == 0:
        print(f"                      Date: {date}\n"
              f"                      Time: {time}\n"
              f"   Prob of bloom appearing: {degree_bloom_appearance}\n"
              f"Prob of bloom disappearing: {degree_bloom_disappearance}")

    count += 1
