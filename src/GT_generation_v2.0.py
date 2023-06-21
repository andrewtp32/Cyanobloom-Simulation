import os
import cv2
import csv
import sys
import time
import codecs
import colorsys
import urllib.error
import urllib.request
import numpy as np
import pandas as pd
import tkinter as tk
import skfuzzy as fuzz
import matplotlib.pyplot as plt
from PIL import Image
from statistics import mean
from bs4 import BeautifulSoup
from tkinter import filedialog
from skfuzzy import control as ctrl
from shapely.geometry import Point
from shapely.geometry import Polygon

# record the start time
start_time = time.time()

# base directory for hotspot data
base_directory = "/Users/andrewphillips/Documents/Documents - Andrew’s MacBook Air/College/PaoloLab/uri_soft_wip/cyanobloom_simulation/jess_sat_data"
# initialize row vectors for the lower and upper HSV ranges.
lower_range_water = np.array([91, 57, 211])
upper_range_water = np.array([100, 63, 234])
# initialize strings for directory and file path
OSM_data_file_path = "/Users/andrewphillips/Documents/Documents - Andrew’s MacBook Air/College/PaoloLab/uri_soft_wip/cyanobloom_simulation/openstreetmap_data_files/SabattusPond.osm"
openstreetmap_file_path = "/Users/andrewphillips/Documents/Documents - Andrew’s MacBook Air/College/PaoloLab/uri_soft_wip/cyanobloom_simulation/openstreetmap_screenshots/SabattusPond.png"
save_folder_path = "/Users/andrewphillips/Documents/Documents - Andrew’s MacBook Air/College/PaoloLab/uri_soft_wip/cyanobloom_simulation/ground_truths/SabattusPond/2016/ground_truth_coordinates"
flight_path_file_path = "/Users/andrewphillips/Documents/Documents - Andrew’s MacBook Air/College/PaoloLab/uri_soft_wip/cyanobloom_simulation/drone_picture_coordinates/SabattusPond"
# Optional start and end dates
# If nothing is specified, the forecast is retrieved.
# If start date only is specified, a single historical or forecast day will be retrieved
# If both start and and end date are specified, a date range will be retrieved
start_date = '2016-05-01'
end_date = '2016-10-31'


class FuzzyBloomModel:
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
    irradiance_flux_6hr['high'] = fuzz.trapmf(irradiance_flux_6hr.universe, [1600, 2100, 8000, 8000])

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

    """
    SECOND LEG OF FUZZY SYSTEM
    """
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

    def __init__(self):
        pass

    @staticmethod
    def first_leg(avg_wind_speed, irradiance_flux_6hr, time_of_day):
        # We can now simulate our control system by simply specifying the inputs and calling the `compute` method.
        # Pass inputs to the ControlSystem using Antecedent labels with Pythonic API
        # Note: if you like passing many inputs all at once, use .inputs(dict_of_data)

        FuzzyBloomModel.stability_state.input['avg_wind_speed'] = avg_wind_speed
        FuzzyBloomModel.stability_state.input['irradiance_flux_6hr'] = irradiance_flux_6hr
        FuzzyBloomModel.stability_state.compute()

        FuzzyBloomModel.buoyancy_state.input['irradiance_flux_6hr'] = irradiance_flux_6hr
        FuzzyBloomModel.buoyancy_state.input['time_of_day'] = time_of_day
        FuzzyBloomModel.buoyancy_state.compute()

        return FuzzyBloomModel.stability_state.output['stability'], FuzzyBloomModel.buoyancy_state.output['buoyancy']

    @staticmethod
    def second_leg(stability, buoyancy):
        # We can now simulate our control system by simply specifying the inputs and calling the `compute` method.
        # Pass inputs to the ControlSystem using Antecedent labels with Pythonic API
        # Note: if you like passing many inputs all at once, use .inputs(dict_of_data)
        # Then crunch the numbers
        # These values are the outputs of the second (and final) leg of the fuzzy systems.

        FuzzyBloomModel.degree_bloom_appearance_state.input['stability_leg2'] = stability  # <- from leg one
        FuzzyBloomModel.degree_bloom_appearance_state.input['buoyancy_leg2'] = buoyancy  # <- from leg one
        FuzzyBloomModel.degree_bloom_appearance_state.compute()

        FuzzyBloomModel.degree_bloom_disappearance_state.input['stability_leg2'] = stability  # <- from leg one
        FuzzyBloomModel.degree_bloom_disappearance_state.compute()

        return FuzzyBloomModel.degree_bloom_appearance_state.output['degree_bloom_appearance'], \
               FuzzyBloomModel.degree_bloom_disappearance_state.output['degree_bloom_disappearance']


class BloomGUI:
    def __init__(self):
        # initialize the color picker image
        self.image = []

        # define the root window
        self.root = tk.Tk()
        # set title and geometry
        self.root.title("Cyanobacterial Bloom Spawning and Evolution Simulation")

        # Set default font sizes
        title_font = ('Arial', 28)
        heading_font = ('Arial', 20)
        body_font = ('Arial', 16)

        # initialize variables for Entries
        self.var_start_date = tk.StringVar(value='2016-05-01')
        self.var_end_date = tk.StringVar(value='2016-10-31')

        # create labels for each input
        self.label_start_date = tk.Label(self.root,
                                         text="Start date (YYYY-MM-DD):",
                                         font=body_font,
                                         padx=5,
                                         pady=10,
                                         anchor="e",
                                         justify="right")
        self.label_end_date = tk.Label(self.root,
                                       text="End date (YYYY-MM-DD):",
                                       font=body_font,
                                       padx=5,
                                       pady=10,
                                       anchor="e",
                                       justify="right")
        self.label_instructions = tk.Label(self.root,
                                           text="Enter your desired values for start/end date and save/load file \n"
                                                "paths. Look over the ReadMe.md file for instructions on gathering \n"
                                                "OpenStreetMap screen captures and coordinate data. Further \n"
                                                "instructions and documentation can be found on GitHub:\n"
                                                "https://github.com/andrewtp32/Cyanobloom-Simulation",
                                           font=heading_font,
                                           padx=5,
                                           pady=10,
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


def generate_gaussian_distribution(mu, num_generated):
    # Cov has a sigma of 15. The dividend is the number of meters in one coordinate length (this basically allows us to
    # scale the distribution)
    cov_matrix = np.array([[225 / 111139 ** 2, 0],
                           [0, 225 / 111139 ** 2]])
    # return a simple bi-variate normal distribution
    return np.random.multivariate_normal(mu, cov_matrix, num_generated)


def find_closest(array, point):
    # stack the two vectors in the shape of a (N x 2)
    array = np.asarray(array)
    # return the index of the closest array value to the coordinate
    return (np.mean(np.abs(array - point), axis=1)).argmin()


def generate_drone_desired_positions_for_images_txt_file(array_of_points):
    # create a shape out of the contour points
    poly = Polygon(array_of_points)
    # boundaries of the shape
    lat_min, lon_min, lat_max, lon_max = poly.bounds
    # Create an mgrid for the values in the x and y directions with an interval of 50 units
    X, Y = np.mgrid[lon_min:lon_max:100, lat_min:lat_max:100]

    # np.ravel() will unravel all elements from a N-dimensional matrix into a 1D array zip() functions return a "zip"
    # object which is an iterator of tuples where the first item in each passed iterator is paired together,
    # and then the second items are pair, and so on. Basically, it takes two arrays, makes them vertical,
    # and slaps them together. list() converts the array zip object to a list np.vstack() simply stacks vectors on
    # top of each other (just as the way it sounds)
    drone_flight_path = np.vstack(list(zip(X.ravel(), Y.ravel())))
    # print the stacked list of
    # print(f"There are {len(drone_flight_path)} points where the drone will take a picture\n")
    flight_path_file = open(f"{flight_path_file_path}/drone_flight_path.txt", "w")
    # save values to a file
    for v in drone_flight_path:
        np.savetxt(flight_path_file, [v], delimiter=',')


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


# run GUI
# get load path for screenshot of water, get the location of the lake, get save path for ground truths and flight path
# BloomGUI()
"""
# create drone flight plan
generate_drone_desired_positions_for_images_txt_file(realistic_contour)
"""
# run API to get weather info for the lake location, convert data into workable arrays
# get API query and convert to CSV
sabattus_coordinate = [44.148657, -70.103106]
CSV_text = create_CSV_from_API(sabattus_coordinate, start_date, end_date)
# create an (n x 5) array. the columns are: [date, time, irradiation, wind speed (m/s), wind direction (degrees)]
weather_data_array = create_data_arr_from_CSV(CSV_text)

# read the csv for the month corresponding to the weather data
full_df = pd.read_csv(f'{base_directory}/hotspots_monthyear.csv')

# create instance of the fuzzy model class
f = FuzzyBloomModel()

# initialize lists and arrays
irradiance_list = []
particle_array = np.array([[0, 0]])

# create plot
fig, ax = plt.subplots(figsize=[6, 8])
ax.set_aspect('equal')

# For each time step, calculate the degree of bloom formation and disappearance:
for index, data_vector in enumerate(weather_data_array):

    # initialize deletion array
    deletion_index_array = np.array([])

    """---------- Keep count of birth and reproduction rate ----------"""
    num_births = 0
    num_reproduced = 0

    """---------- Make the weather data more workable ----------"""
    # create easy-to-read variable names for the elements in the data vector
    date = data_vector[0]
    time_ = int(data_vector[1])
    irradiance = float(data_vector[2])
    wind_speed = float(data_vector[3])
    wind_direction = float(data_vector[4])

    # extract the year and month values from the "date"
    year = int(date[:4])
    month = int(date[5:7])
    day = int(date[8:])

    # append irradiance to the list
    irradiance_list.append(irradiance)
    # delete the unnecessary items in the irradiance list so that we only hold the last 6 hurs
    if len(irradiance_list) > 6:
        irradiance_list = irradiance_list[-6:]
    # calculate the irradiance sum over the last 6 hours
    irradiance_over_last_6hrs = sum(irradiance_list)

    """---------- Input weather data into fuzzy logic system ----------"""
    # determine the degree of bloom appearance and disappearance. the fuzzy logic system is completed in two legs.
    stability_first_leg, buoyancy_first_leg = f.first_leg(wind_speed, irradiance_over_last_6hrs, time_)
    degree_bloom_appearance, degree_bloom_disappearance = f.second_leg(stability_first_leg, buoyancy_first_leg)

    # print(f"{index}\n"
    #       f"Deg app: {degree_bloom_appearance}\n"
    #       f"Deg dis: {degree_bloom_disappearance}")

    """---------- Format Jess' data ----------"""
    # locate relevant satellite data
    month_df = full_df.loc[(full_df['Year'] == year) & (full_df['Month'] == month)]
    # pull out just the coordinates
    month_df_coordinates = month_df[['x', 'y']].copy()
    # pull out just the extremely hot data points, then just to coordinates
    month_df_hot = month_df.loc[(month_df['labels'] == 4)]
    month_df_hot = month_df_hot[['x', 'y']].copy()
    # convert dataframe to a numpy object
    month_df.to_numpy()
    month_df_coordinates.to_numpy()
    month_df_hot.to_numpy()
    # make dataframe into a numpy array
    month_data_arr = np.array(month_df)
    month_coordinates_arr = np.array(month_df_coordinates)
    month_hot = np.array(month_df_hot)

    """---------- Migrate the particles based on wind speed and direction ----------"""
    # particle movement (multiply by some slope and angle, i.e., the wind speed and direction from data)
    wnd_damp_param = 0.00005
    if len(particle_array) > 1:
        arr_x, arr_y = zip(*particle_array)
        particle_array = np.column_stack((arr_x + wnd_damp_param * (np.add(-0.5, np.random.rand(len(arr_x))) +
                                                                    wind_speed * np.cos(wind_direction * np.pi / 180)),
                                          arr_y + wnd_damp_param * (np.add(-0.5, np.random.rand(len(arr_y))) +
                                                                    wind_speed * np.sin(wind_direction * np.pi / 180))))

    """---------- Particle spawn ----------"""
    # particle spawn
    # if deg bloom dis is appropriate and jess's data is also appropriate, then spawn particle
    spawning_param = 75
    if degree_bloom_appearance > spawning_param:
        # append the HOT coordinates if the degree of formation is high enough
        particle_array = np.append(particle_array, month_hot, axis=0)
        num_births += len(month_hot)

    """---------- Rearrange, Reproduce, Kill particles ----------"""
    if len(particle_array) > 1:
        # loop through the array of particles
        for i, particle in enumerate(particle_array):
            # find the coordinate that most closely correlates to the particle in question
            closest_coordinate_index = find_closest(month_coordinates_arr, particle)
            # pull the row in Jess's data with that index
            hotspot_data_at_coordinate = month_data_arr[closest_coordinate_index]

            # rearrange particle
            # check if the particle id out of the range of the water body
            if np.mean(abs(particle - [hotspot_data_at_coordinate[0], hotspot_data_at_coordinate[1]])) > 0.0001:
                # if outside of body, assign the point's location back to the edge of the body
                particle_array[i] = [hotspot_data_at_coordinate[0], hotspot_data_at_coordinate[1]]

            # reproduce particles
            # if jess data is "not significant" and the deg bloom appearance is very high, then reproduce
            if (hotspot_data_at_coordinate[5] == 2) and (degree_bloom_appearance >= 75):
                # append the reproduced particles
                particle_array = np.append(particle_array, generate_gaussian_distribution(particle, 1), axis=0)
                num_reproduced += 1
            # if jess data is "hot" and the deg bloom appearance is high or very high, then reproduce
            if (hotspot_data_at_coordinate[5] == 3) and (degree_bloom_appearance >= 45):
                # append the reproduced particles
                particle_array = np.append(particle_array, generate_gaussian_distribution(particle, 1), axis=0)
                num_reproduced += 1

            # kill particles
            # if jess data is "not significant" and the deg bloom disappearance is very high, then kill
            if (hotspot_data_at_coordinate[5] == 2) and (degree_bloom_disappearance >= 75):
                # append the index to an array
                deletion_index_array = (np.append(deletion_index_array, i)).astype(int)
            # if jess data is "cold" and the deg bloom disappearance is high or very high, then kill
            if (hotspot_data_at_coordinate[5] == 1) and (degree_bloom_disappearance >= 45):
                # append the index to an array
                deletion_index_array = (np.append(deletion_index_array, i)).astype(int)
            # if jess data is "very cold" and the deg bloom disappearance is moderate, high, or very high, then kill
            if (hotspot_data_at_coordinate[5] == 0) and (degree_bloom_disappearance >= 20):
                # append the index to an array
                deletion_index_array = (np.append(deletion_index_array, i)).astype(int)

    """---------- Kill off the categorized particles ----------"""
    if len(deletion_index_array) > 1:
        # remove the particles from the array
        particle_array = np.delete(particle_array, deletion_index_array, 0)

    print(f"Date: {month}/{day}/{year}, {time_}")
    print(f"Total particles: {len(particle_array)}")
    print(f"Num births in latest step: {num_births}")
    print(f"Num reproduced in latest step: {num_reproduced}")
    print(f"Num deaths in latest step: {len(deletion_index_array)}\n")

    ground_truths_x, ground_truths_y = zip(*particle_array)
    month_coordinates_arr_x, month_coordinates_arr_y = zip(*month_coordinates_arr)
    # create figure for each time step
    ax.set_title(f"{month}/{day}/{year}, {time_}")
    ax.scatter(month_coordinates_arr_x, month_coordinates_arr_y, color="blue")
    ax.scatter(ground_truths_x[1:], ground_truths_y[1:], color="green", s=0.5)
    plt.pause(0.5)
    plt.cla()

# print the total time to complete program
print("\n--- %s seconds ---" % (time.time() - start_time))
