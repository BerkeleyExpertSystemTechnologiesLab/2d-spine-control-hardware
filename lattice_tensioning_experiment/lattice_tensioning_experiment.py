#!/usr/bin/python

# NOTE: on Ubuntu 18.04, specifying /usr/bin/env python uses python 3,
# while specifying /usr/bin/python uses python 2.7. We need 2.7.

# lattice_tensioning_experiment compares pictures of Belka versus inverse statics.
# Usage:
#	./lattice_tensioning_experiment.py path-to-image-file

# Imports:
# because we need the command-line arguments
import sys
# image processing might as well use opencv
import cv2
import numpy as np

# The primary helper function here opens the serial device,
# and iteratively reads lines from it until stopped.
# frustratingly enough, hardware interrupts are difficult on linux, so we poll the device at some interval

def run_experiment(img_path):
    print("Running lattice tensioning experiment with image file: " + img_path)
    img = cv2.imread(img_path, cv2.IMREAD_COLOR)
    # Raw from the DSLR is a huge image
    scale_percent = 30 # percent of original size. 30 works with a 1200x1600 monitor
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    # resize image
    img_resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA) 
    cv2.imshow("Belka Lattice Tensioning Experiment", img_resized)
    cv2.waitKey(0)

# the main function: just call the helper, while parsing the serial port path.
if __name__ == '__main__':
    try:
        print('Lattice Tensioning Experiment for Belka. Copyright 2020 Andrew Sabelhaus / BEST lab.')
        img_filepath = ""
        # Less typing = hard-coded path. Comment out to get a prompt
        img_filepath = "belka_doublelattice_oct2020.jpg"
        if not img_filepath:
            # the 0-th arg is the name of the file itself, so we want the 1st if it exists
            if len(sys.argv) < 2:
                img_filepath = input("Path to image file: ")
            else:
                img_filepath = sys.argv[1]
        run_experiment(img_filepath)
    except KeyboardInterrupt:
    	# why is this here?
        pass
