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
# our calibration package
import calculate_homography2

# helper to hard-code the coordinates we'll click on for the calibration
def get_calibration_pts():
    # Assuming four points for now.
    xy = np.ndarray((4, 2))
    # number of dots/grid marks and distance between them = total distance
    num_marks = 20
    mark_dist = 2.54 # here's our units, cm. From 9/28, Dorotea's backdrop had 1 inch
    edge_dist = num_marks*mark_dist
    # Clicks go upwards then clockwise from (0,0)
    xy[0,:] = [0, 0]
    xy[1,:] = [0, edge_dist]
    xy[2,:] = [edge_dist, edge_dist]
    xy[3,:] = [edge_dist, 0]
    return xy

# Callback function for 'cv2.SetMouseCallback' adds a clicked point to the
# list 'points'
def on_mouse_click(event,x,y,flag,param):
    if(event == cv2.EVENT_LBUTTONUP):
        point = (x,y)
        print "Point Captured: " + str(point)
        param.append(point)

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
    window_name = "Belka Lattice Tensioning Experiment"
    cv2.imshow(window_name, img_resized)

    # Take desired number of clicks from the hard-coded global coordinates
    calib_pts = get_calibration_pts()
    n_calib_clicks = calib_pts.shape[0]
    print("Please click " + str(n_calib_clicks) + " times on the image.")
    homography_clicks = []
    cv2.setMouseCallback(window_name, on_mouse_click, param=homography_clicks)
    # First N-many clicks are for calibration
    while len(homography_clicks) < n_calib_clicks:
        cv2.waitKey(10)
    print("Got " + str(n_calib_clicks) + " calibration points.")

    # Now, calculate the transformation between camera frame and global frame.
    # Convert the Python list of points to a NumPy array of the form
    #   | u1 u2 u3 u4 |
    #   | v1 v2 v3 v4 |
    # ^^^ACTUALLY IT'S TRANSPOSED, points are rows
    uv = np.array(homography_clicks)
    H = calculate_homography2.calc_H(uv, calib_pts)
    print("Homography matrix: ")
    print(H)

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
