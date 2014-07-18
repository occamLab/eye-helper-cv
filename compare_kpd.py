import cv2
import numpy as np 
import csv
import pickle
import scipy 
import os
from objectmatch import mean_shift

def calc_center(g_truth):
    """ 
        Function finds the center of a box

        Inputs: g_truth -> four corners of a box
        Returns: center -> the center of the box
    """

    x = int((g_truth[2]+g_truth[0])/2)
    y = int((g_truth[3]+g_truth[1])/2)
    return x, y

def superdata(q_pickle, t_pickle, g_truth, frame, method, t_img):
    """
        we may or may not rename this function in the near future to be something more descriptive... 
        --emily and lindsey, july 18, 2014

        Inputs:
            q_pickle -> keypoint and descriptor data from query image
            t_pickle -> keypoint and descriptor data from training image
            g_truth -> coordinates of labeled box around actual object
            frame -> frame number
            method -> kp detector method used to get points
            t_img -> training image frame number

        We assume that the object has been perfectly selecdted so we use ground truth values for the training image region.

        Returns:
            c_center -> center of the object tracking circle
            b_center -> center of the object
            match -> boolean True or False
            frame -> the number of the frame
            t_frame -> number of the training image
            kp_matches -> total number of matched keypoints
            c_matches -> number of correctly matched keypoints
    """
    #load query image data
    q = pickle.load(open(q_pickle, 'rb'))
    q_k = q[0]
    q_d = q[1]

    #load training image data
    t = pickle.load(open(t_pickle, 'rb'))
    t_k = t[0]
    t_d = t[1]

    #iterate through the training keypoints and only keep those that are within the selected object
    #In this case we are using ground truth as the selection
    train_d = []
    train_k =[]
    for index in range(len(t_k)):
        if g_truth[2]<=t_k[index][0]<=g_truth[0] and g_truth[3]<=t_k[index][1]<=g_truth[1]:
            train_d.append(t_d[index])
            train_k.append(t_k[index])
    t_k = train_k
    t_d = np.array(train_d)

    #Selects the correct matcher parameters depending on the keypoint detector used
    if method == 'SIFT' or method == 'SURF':
        bf = cv2.BFMatcher()
    else: #for ORB or BRISK
        bf = cv2.BFMatcher(normType = cv2.NORM_HAMMING)

    #matches keypoints of *training* image to query, not other way around
    matches = bf.knnMatch(t_d, q_d, k=2)

    good_matches = []
    #goes through and only keeps matches passing a nearest neighbor test to reduce false matches
    for m,n in matches:
        if m.distance < 0.75*n.distance:
            # Get coordinate of the match
            m_x = int(q_k[m.trainIdx][0])
            m_y = int(q_k[m.trainIdx][1])
            good_matches.append((m_x, m_y))

    #uses mean_shift to find the object from the keypoints detected
    c_center, radius = mean_shift(hypothesis = (100, 100), keypoints = good_matches, threshold = 10, frame = frame)
    b_center = calc_center(g_truth)
    
    kp_matches = len(good_matches)
    c_matches = 0
    for match in good_matches:
        if g_truth[2]<=match[0]<=g_truth[0] and g_truth[3]<=match[1]<=g_truth[1]:
            c_matches +=1
    
    if g_truth[2]<=c_center<=g_truth[0] and g_truth[3]<=c_center<=g_truth[1]:
        match = True
    else:
        match = False

    return c_center, b_center, match, frame, t_img, kp_matches, c_matches

if __name__ == '__main__':

    print superdata(q_pickle = './OT-res/kp_pickles/cookie/SIFT/cookie_00107_keypoints.p', 
                    t_pickle = './OT-res/kp_pickles/cookie/SIFT/cookie_00177_keypoints.p', 
                    g_truth = [744,514,606,392], 
                    frame = 177, 
                    method = 'SIFT', 
                    t_img = 177)