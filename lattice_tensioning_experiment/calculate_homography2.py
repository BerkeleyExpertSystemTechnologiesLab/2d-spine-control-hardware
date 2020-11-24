#!/usr/bin/python2.7

# This module has one function that calculates the homography matrix for a given scene,
# given four points.
# This code is based on Drew Sabelhaus and Saunon Malekshahi's implementation
# of EE206A's lab4 from fall of 2018.
# UPDATE: Modifed by Jacob Madden in spring of 2019

# imports:
import numpy as np
from numpy.linalg import lstsq
import cv2

def calc_H(uv, xy):
    # Note: uv are the clicked points (camera frame) and xy are the global points
    num_pts = np.size(uv, 0)
    # Initialize the A and b matrices
    # There will be 2*N rows, since each point has an x, y position
    A = np.zeros((2 * num_pts, 8))
    # The b vector will be 2N long
    b = np.zeros((2 * num_pts, 1))
    h = np.zeros((8, 1))

    # print b2

    # print 'A init: ' + str(A)
    # print np.size(A, 0)
    # print 'b init: ' + str(b)
    # print np.size(b, 0)

    # Loop through and build up A and b based on the points matrix.
    for i in range(num_pts):

        A[i * 2, :] = [xy[i, 0], xy[i, 1], 1, 0, 0, 0, -uv[i, 0] * xy[i, 0], -uv[i, 0] * xy[i, 1]]
        A[i * 2 + 1, :] = [0, 0, 0, xy[i, 0], xy[i, 1], 1, -uv[i, 1] * xy[i, 0], -uv[i, 1] * xy[i, 1]]
        b[i * 2, 0] = uv[i, 0]
        b[i * 2 + 1, 0] = uv[i, 1]

    # Find homography vector, h, by using least squares minimization
    # print 'A: ' + str(np.int64(A))
    # print 'b: ' + str(b)
    # Ainv = np.linalg.inv(A)
    # h = np.dot(Ainv, b)
    h_soln = np.linalg.lstsq(A, b)
    h = h_soln[0]

    # h is of the form [h11, h12, h13, h21, h22, h23, h31, h32]
    # but each element is its own one-element array,
    # so we double-index here
    H = np.array([[h[0, 0], h[1, 0], h[2, 0]],
                  [h[3, 0], h[4, 0], h[5, 0]],
                  [h[6, 0], h[7, 0], 1]])

    return(H)