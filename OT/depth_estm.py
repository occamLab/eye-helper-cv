import cv2
import numpy as np 
import matplotlib.pyplot as plt 


def find_depth(img1, img2):
    """
    Goal:
        calculated disparity map
        From disparity map estimate depth of image objects

    Inputs:
        img1 -> file path to left image
        img2 -> file path to right image

    Returns:
        estimated depth map 
    """

    left = cv2.imread(img1, 0)
    right = cv2.imread(img2, 0)

    stereo = cv2.StereoBM(preset = 0, ndisparities=208, SADWindowSize = 5)#minDisparity=0, numDisparities=192, SADWindowSize=7)
    disparity = stereo.compute(left,right)

    cv2.imshow('right',right)
    cv2.imshow('disparity', disparity)
    cv2.waitKey(0)

if __name__ == '__main__':
    find_depth('../gstore_snippets/cookie_snippet/cookie_00124.jpg', '../gstore_snippets/cookie_snippet/cookie_00130.jpg')

