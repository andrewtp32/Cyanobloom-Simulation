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
https://github.com/visualcrossing/WeatherApi

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
import matplotlib.pyplot as plt
from numpy import genfromtxt
from tkinter import filedialog
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
openstreetmap_file_path = ''
save_folder_path = ''
flight_path_file_path = ''
# Location for the weather data
location = ''
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
        self.var_coordinates_of_water_body = tk.StringVar(value='41.501953,-71.564331')
        self.var_start_date = tk.StringVar(value='2022-04-20')
        self.var_end_date = tk.StringVar(value='2022-10-20')

        # create labels for each input
        self.label_coordinates_of_water_body = tk.Label(self.root,
                                                        text="Coordinates of water body (no spaces):",
                                                        font=body_font, padx=5, pady=10, anchor="e",
                                                        justify="right")
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
        self.entry_coordinates_of_water_body = tk.Entry(self.root, textvariable=self.var_coordinates_of_water_body,
                                                        font=body_font)
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
                                                                  text="Choose save path for drone\nflight path coordinates",
                                                                  command=self.command_choose_flight_path_save_file_path,
                                                                  padx=5,
                                                                  pady=10)
        self.button_pick_water_area_color = tk.Button(self.root,
                                                      text="Click on an area on the picture that\nyou want to denote as 'water'",
                                                      command=self.command_pick_water_color,
                                                      padx=5,
                                                      pady=10)

        # create text boxes
        self.textbox_save_directory_path = tk.Text(self.root, font=body_font, height=4, width=50)
        self.textbox_open_file_path = tk.Text(self.root, font=body_font, height=4, width=50)
        self.textbox_choose_flight_path_save_file_path = tk.Text(self.root, font=body_font, height=4, width=50)
        self.textbox_pick_water_area_color = tk.Text(self.root, font=body_font, height=2, width=50)

        # Initialize text boxes
        self.textbox_save_directory_path.insert(tk.END, os.getcwd())
        self.textbox_open_file_path.insert(tk.END, os.getcwd())
        self.textbox_choose_flight_path_save_file_path.insert(tk.END, os.getcwd())

        # apply things to the grid. the grid is set up to be (9x3)
        # column 0
        self.label_instructions.grid(row=0, column=0, columnspan=3)
        self.label_coordinates_of_water_body.grid(row=1, column=0, sticky=tk.E)
        self.label_start_date.grid(row=2, column=0, sticky=tk.E)
        self.label_end_date.grid(row=3, column=0, sticky=tk.E)
        self.button_choose_openstreetmap_file.grid(row=4, column=0, sticky=tk.E)
        self.button_pick_water_area_color.grid(row=5, column=0, sticky=tk.E)
        self.button_choose_save_path.grid(row=6, column=0, sticky=tk.E)
        self.button_choose_flight_path_save_file_path.grid(row=7, column=0, sticky=tk.E)
        # column 1
        self.entry_coordinates_of_water_body.grid(row=1, column=1, columnspan=2, sticky=tk.W)
        self.entry_start_date.grid(row=2, column=1, columnspan=2, sticky=tk.W)
        self.entry_end_date.grid(row=3, column=1, columnspan=2, sticky=tk.W)
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

    def command_submit_entries(self):
        global location
        global start_date
        global end_date
        # convert the entries into workable variables
        location = self.var_coordinates_of_water_body.get()
        start_date = self.var_start_date.get()
        end_date = self.var_end_date.get()

        # close window
        self.root.destroy()

    def command_remove_entries(self):
        # reset the entries
        self.entry_coordinates_of_water_body.delete(0, tk.END)
        self.entry_start_date.delete(0, tk.END)
        self.entry_end_date.delete(0, tk.END)
        self.textbox_open_file_path.delete('1.0', 'end')
        self.textbox_save_directory_path.delete('1.0', 'end')
        self.textbox_pick_water_area_color.delete('1.0', 'end')

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


def is_leap_year(year):
    """ if year is a leap year return True
        else return False """
    if year % 100 == 0:
        return year % 400 == 0
    return year % 4 == 0


def enumerate_date(date):
    # enumerate the start date in reference to the entire year
    # strip the date to year, month, and day values
    year = int(date[0:4])
    month = int(date[5:7])
    day = int(date[8:])
    if is_leap_year(year):
        K = 1
    else:
        K = 2
    enumerated_date = int((275 * month) / 9.0) - K * int((month + 9) / 12.0) + day - 30
    return enumerated_date


def get_API_query(loc, s_date, e_date):
    # This is the core of our weather query URL
    base_URL = 'https://weather.visualcrossing.com/VisualCrossingWebServices/rest/services/timeline/'
    # my API key found at: https://www.visualcrossing.com/account
    API_Key = 'S4ZCGVSBWEJY4SS3F6W3QKGZH'
    # UnitGroup sets the units of the output - us or metric
    unit_group = 'us'
    # JSON or CSV
    # JSON format supports daily, hourly, current conditions, weather alerts and events in a single JSON package
    # CSV format requires an 'include' parameter below to indicate which table section is required
    content_type = "csv"
    # include sections
    # values include days,hours,current,alerts
    include = "days"
    # basic query including location
    api_query = base_URL + loc
    # append the start and end date if present
    if len(s_date):
        api_query += "/" + s_date
        if len(e_date):
            api_query += "/" + e_date
    # Url is completed. Now add query parameters (could be passed as GET or POST)
    api_query += "?"
    # append each parameter as necessary
    if len(unit_group):
        api_query += "&unitGroup=" + unit_group
    if len(content_type):
        api_query += "&contentType=" + content_type
    if len(include):
        api_query += "&include=" + include
    # add API key
    api_query += "&key=" + API_Key

    return api_query


def open_URL_and_parse_results(api_query):
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
    CSV_text = csv.reader(codecs.iterdecode(CSV_bytes, 'utf-8'))
    return CSV_text


def create_list_of_temps(CSV_file):
    # The first row contains the headers and the additional rows each contain the weather metrics for a single day
    # We will ignore the first row, then append all 'temp' and 'feelslike' values to a list
    # 'temp' is the 4th index and 'feelslike' is the 7th index
    # before appending to the list, covert the resulting values into strings
    feels_like_temp_array = [float(col) for index_row, row in enumerate(CSV_file) for index_col, col in
                             enumerate(row) if index_row > 0 if index_col == 7]
    # If there are no CSV rows then something fundamental went wrong
    if len(feels_like_temp_array) == 0:
        print('Sorry, but it appears that there was an error connecting to the weather server.')
        print('Please check your network connection and try again..')
    # If there is only one CSV row then we likely got an error from the server
    if len(feels_like_temp_array) == 1:
        print('Sorry, but it appears that there was an error retrieving the weather data.')
        print('Error: ', CSV_file)

    return feels_like_temp_array


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


def get_water_contours_and_drone_flight_path(lower_range, upper_range):
    # get image from file path
    img = cv2.imread(openstreetmap_file_path)

    # initialize a list of valid water area contour points
    extracted_water_area_contour_points_array = []
    # initialize a list for vertexes of bounding box
    bounding_box_info = []
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

    # calculate perimeter of the contour
    peri = cv2.arcLength(main_water_contour, True)
    # make the shape of the contour simpler by approximation
    approx = cv2.approxPolyDP(main_water_contour, 0.02 * peri, True)
    # calculate a bounding box for the contour
    x, y, width, height = cv2.boundingRect(approx)
    bounding_box_info = [x, y, width, height]
    # create drone flight plan
    generate_drone_desired_positions_for_images_txt_file(x, y, width, height)

    # transforming data into a 2d array
    for point in main_water_contour:
        # print(f"'point' in 'cnt': {point}")
        extracted_water_area_contour_points_array.append(point[0])

    return extracted_water_area_contour_points_array, bounding_box_info


# run GUI
# get load path for screenshot of water, get the location of the lake, get save path for ground truths and flight path
BloomGUI()

# run API to get weather info for the lake location, convert data into workable arrays
# get API query
API_query = get_API_query(location, start_date, end_date)
# open URL and check for opening errors
CSV_text = open_URL_and_parse_results(API_query)
# create list of temperatures (one for each day)
feels_like_temp_arr = create_list_of_temps(CSV_text)
print(len(feels_like_temp_arr))

# get contours for the water area, get the coordinates for the bounding box
water_contours, contour_bounding_box = get_water_contours_and_drone_flight_path(lower_range=lower_range_water,
                                                                                upper_range=upper_range_water)

# initialize estimated water temp arr
water_temp_estimate_arr = []
# enumerate the start date in reference to the entire year
enumerated_start_date = enumerate_date(start_date)

# For each time step:
for index, atmospheric_temp in enumerate(feels_like_temp_arr):
    # calculate current day number
    day_number = enumerated_start_date + index
    print(f"Day number: {day_number}")
    # constants
    temp_max = 78  # highest water temp of lakes on average
    temp_min = 32  # lowest water temp possible
    temp_amplitude = (temp_max - temp_min) / 2
    period = 2 * np.pi / 365
    day_shift = 207  # warmest day of year is usually july 26th
    amplitude_shift = temp_min + temp_amplitude
    # create estimate of water temperature
    water_temp_estimate = temp_amplitude * np.sin(period * (day_number - day_shift)) + amplitude_shift
    print(f"Estimate of water temp: {water_temp_estimate}")
    # append estimate to an arr
    water_temp_estimate_arr.append(water_temp_estimate)
    # calculate probabilities for blooms to spawn, reproduce, die, or do nothing based on temp data

    # use probabilities for the actual action of spawn, death, reproduce, exist
    # move particles back to within the water boundary
    # use wind and vortex vectors to migrate the particles
    # save array of points to a csv file

# plot all of the time steps
plt.figure()
plt.scatter(np.linspace(0, len(water_temp_estimate_arr)), water_temp_estimate_arr, color="blue", s=1.5)
plt.show()
