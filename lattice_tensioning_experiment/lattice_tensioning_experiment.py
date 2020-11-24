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
    num_marks = 10
    mark_dist = 2.54 # here's our units, cm. From 9/28, Dorotea's backdrop had 1 inch
    edge_dist = num_marks*mark_dist
    # Clicks go downwards then counterclockwise from (0,0)
    # We'll do a rectangle that's twice as long width-wise for better perspective
    xy[0,:] = [0, 0]
    xy[1,:] = [0, -edge_dist]
    xy[2,:] = [2*edge_dist, -edge_dist]
    xy[3,:] = [2*edge_dist, 0]
    return xy

# also for now, hard-code the coordinates from the inverse statics calculations
def get_is_pts():
    xy = np.array([[0,0], [14.2, 2.25], [28.4, 3], [42.6, 2.25], [56.8, 0]])
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
    # We use the inverse of this matrix to convert points.
    Q = np.linalg.inv(H)
    print("Inverse transform: ")
    print(Q)

    # Finally, click on the vertebrae/bodies to track.
    n_bodies = 5
    print("Please click on " + str(n_bodies) + " bodies you'd like to know the position of.")
    robot_clicks = []
    cv2.setMouseCallback(window_name, on_mouse_click, param=robot_clicks)
    while len(robot_clicks) < n_bodies:
        cv2.waitKey(10)
    print("Got " + str(n_bodies) + " points on the robot.")
    robot_pts = np.array(robot_clicks).T
    # Each in the global frame:
    for i in range(0, n_bodies):
        # homogenous coordinates in 2D are 3x1 column vectors
        body_i = np.r_[robot_pts[:, i], 1]
        global_i = np.dot(Q, body_i)[0:2]
        print("Body " + str(i) + ": Camera frame =" + str(robot_clicks[i]) + ", Global frame =" + str(global_i))
    
    # Compare vs. the supposed known points
    is_pts = get_is_pts()
    camera_ref_pts = []
    for i in range(0, is_pts.shape[0]):
        is_pt_i = np.r_[is_pts[i,:].T, 1]
        camera_i = np.dot(H, is_pt_i)[0:2]
        camera_ref_pts.append(camera_i)
        print("Reference Body " + str(i) + ": Global frame = " + str(is_pt_i) + ", Camera frame = " + str(camera_i))
    
    # Plot the points on this image.
    clicked_color = (255, 0, 0)
    clicked_radius = 20 # pixels
    reference_color = (0, 255, 0)
    for i in range(0, n_bodies):
        cv2.circle(img_resized, robot_clicks[i], clicked_radius, clicked_color, -1)
        ref_pt_i = tuple(camera_ref_pts[i].astype(int))
        cv2.circle(img_resized, ref_pt_i, clicked_radius, reference_color, -1)
    cv2.imshow(window_name, img_resized)
    print("Points now displayed. Please close when done.")
    
    # pause until we're done saving this picture, manually.
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
