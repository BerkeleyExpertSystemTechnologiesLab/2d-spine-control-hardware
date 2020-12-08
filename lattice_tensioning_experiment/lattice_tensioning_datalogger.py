#!/usr/bin/env python

# This data logger saves results from the lattice tensioning test.
# Andrew Sabelhaus 12/7/2020

import sys
# for brevity
from datetime import date
from datetime import time
from datetime import datetime
import numpy as np


# Some functions to calculate timestamps
# This one returns a string of the date and starting time, for use
# with file naming.
def get_datetime_string():
    # Should be of the form
    # Year-Month-Day_HourMinSec
    t = date.today()
    stampdate = str(t.year) + "-" + str(t.month) + "-" + str(t.day)
    # Now, for the time itself
    now = datetime.now()
    stamptime = now.strftime("%H%M%S")
    # the full datetime timestamp is
    stamp = stampdate + "_" + stamptime
    return stamp

# The function that actually logs the data.
# filename should be output returned from datalogger_startup,
# ref is list x n_bodies of 1d np array, theoretical points, obs is same shape observed from clicking
def save_test(filename, ref, obs):
    # Open the file, append mode, and binary open so we can use savetxt
    f = open(filename, 'ab')
    # Turn the array into a comma separated string
    # Thanks to https://stackoverflow.com/questions/2721521/fastest-way-to-generate-delimited-string-from-1d-numpy-array
    # We do reference coordinates first then observed coordinates
    x_coord_ref = np.char.mod('%f', ref[0])
    y_coord_ref = np.char.mod('%f', ref[1])
    x_coord_obs = np.char.mod('%f', obs[0])
    y_coord_obs = np.char.mod('%f', obs[1])
    # combine w/coordinate label. needs a newline too
    # First row is x-coordinates, second row is y-coordinates.
    row_x = "x:," + ",".join(x_coord_ref) + "," + ",".join(x_coord_obs) + '\n'
    row_y = "y:," + ",".join(y_coord_ref) + "," + ",".join(y_coord_obs) + '\n'
    # write the file. It may have been easier to use np.savetxt but whatever
    f.write(row_x)
    f.write(row_y)
    print('Saved test results.')
    f.close()

# a helper function for startup, must be called first
def datalogger_startup(file_folder, n_bodies):
    # Create the filename. It's a concatenation of the folder with a descriptive name,
    # and a timestamp.
    filename = file_folder + "/lattice_tensioning_datalogger_" + get_datetime_string() + ".csv"
    print("Opened log file " + filename + ".")
    # Open the file and put a header.
    f = open(filename, 'w')
    f.write("Lattice Tensioning experiment on:," + get_datetime_string())
    f.write("\n")
    # We will have n_bodies worth of comparison points between reference (theory) and observation (experiment). 
    # Each has an x, y coordinate, but we'll do that as column vectors r = [x; y]
    f.write("Coordinate:")
    # to play nice with numpy array-to-csv conversion later, list all the reference first then all the observations after
    for i in range(0, n_bodies):
        f.write(",r_" + str(i) + "_ref") 
    for i in range(0, n_bodies):
        f.write(",r_" + str(i) + "_obs") 
    f.write("\n")
    f.close()
    # to pass in to these helpers later...
    return filename
