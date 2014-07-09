"""
A script for visualizing feature matches between a train image and a query image.
"""

import cv2
import numpy as np
from matplotlib import pyplot as plt
import scipy as sp

def visualize(img_t, img_q, box, good_matches, t_k, q_k):
    """ 
    Displays the ground truth box and matches on the query image

        inputs: training image, query image, box corners, list of good matches,
                training keypoints, query keypoints

        outputs: image with points and box plotted
    """
    # Input images
    img1 = cv2.imread(img_q,0) # queryImage
    img2 = cv2.imread(img_t,0) # trainImage

    # For displaying the matching circles
    h1, w1 = img1.shape[:2]
    h2, w2 = img2.shape[:2]
    view = sp.zeros((max(h1, h2), w1 + w2, 3), sp.uint8)
    view[:h1, :w1, 0] = img1
    view[:h2, w1:, 0] = img2
    view[:, :, 1] = view[:, :, 0]
    view[:, :, 2] = view[:, :, 0]

    for m in good_matches:
        # draw the keypoints
        cv2.circle(view, (int(q_k[m.queryIdx].pt[0]), int(q_k[m.queryIdx].pt[1])), 2, [0, 0, 255], 2)
    
    #box is a list of corners forming a box
    cv2.rectangle(view, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), [255, 0,0]) 

    # Showing the matches
    cv2.imshow("view", view)
    cv2.waitKey(0)

