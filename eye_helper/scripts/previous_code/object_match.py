from __future__ import division
import cv2
import numpy as np 
import scipy as sp 
import csv
import pandas
import subprocess
import threading
import time
import Queue

"""
Object matching shenanigans with meanshift. 

--Emily and Lindsey, July 25, 2014
"""


def audio_loop(queue):
    filename = ''
    while True:
        time.sleep(0.5)
        center = queue.get(block=False)
        if center != None:
            filename = play_audio(center, filename)
            print filename

def draw_circles(img, c, radius, kp=None):
    """Takes in:
        img -> opened numpy array of image
        kp -> list of the keypoint coordinates 
        c -> center of 'mass' of keypoints
        radius -> estimated radius of object
    """
    if kp != None:
        for k in kp:
            cv2.circle(img, (k[0], k[1]), 1, [255, 0, 0], 2)
    # cv2.circle(img, c, radius, [100,255,0], 2)
    if c != None:
        cv2.circle(img, c, 6, [0,0,255], 6)
    # return img

def match_object(previous, current, train_img, pos, frame=0, show = False, live = False, t = ()):
    """
    Takes in:
        previous -> center of the previous image
        current ->  image being analyzed (if not live: the img path)
        train_img -> training image for keypoint matching
        pos -> corners (left top, right bottom) of object in training img
        show -> determines if visualization is displayed
        t -> passes in keypoints and descriptors of training image IF we're using the live_meanshift.py file
    Output:
        new_center -> new center determined by meanshift
                      if there are no matches, original center is returned instead

    """

    # corners of selected item in training image, accounts for the flipping
    x1 = pos[2]
    y1 = pos[3]
    x2 = pos[0]
    y2 = pos[1]

    #I choose YOU! SIFT-emon
    detector = cv2.SIFT()

    if not live:
        #Take in training image with coordinates of tracked object
        t_img = cv2.imread(train_img)

        #query image
        q_img = cv2.imread(current)

        #Keeps only keypoints within the grocery item selection
        t_k, t_d = detector.detectAndCompute(t_img, None)   #training image

        train_d = []
        train_k = []

        for index in range(len(t_k)):
            if x1<=t_k[index].pt[0]<= x2 and y1<= t_k[index].pt[1] <=y2:
                train_k.append(t_k[index])
                train_d.append(t_d[index])
        t_k = train_k
        t_d = train_d


    else:
        #to account for how the webcam version of this deals with inputs differently
        # t_img = train_img #should be cap.read()[1] 
        q_img = current
        t_k = t[0] #keypoints = locations (think x, y coords)
        t_d = t[1] #descriptors = 128-elemehnt vectors (via SIFT) that gives information 
                   #about gradient of surrounding change (changes in intensity) and orientation

        train_d = []
        train_k = []

        for index in range(len(t_k)):
            if x1<=t_k[index][0]<= x2 and y1<= t_k[index][1] <=y2:
                train_k.append(t_k[index])
                train_d.append(t_d[index])
        t_k = train_k
        t_d = train_d

    #finds all the keypoints in the query image
    q_k, q_d = detector.detectAndCompute(q_img, None)   #query image

    try:
        matcher = cv2.BFMatcher() #(normType = cv2.NORM_HAMMING)

        #match list of object keypoints to query image, not other way around
        matches = matcher.knnMatch(np.array(t_d), q_d, k =2)

        #Nearest neighbor test to reduce false matches
        good_matches = []
        # b/c k=2....
        # m is the nearest match
        # n is the next nearest match
        for m,n in matches:
            if m.distance < 0.75*n.distance:
                # Get coordinate of the match
                m_x = int(q_k[m.trainIdx].pt[0])
                m_y = int(q_k[m.trainIdx].pt[1])
                good_matches.append((m_x, m_y))

        # print 'length of good_matches %d' % len(good_matches)

        new_center, current = mean_shift(hypothesis = (previous), 
                                                    keypoints = good_matches, 
                                                    threshold = 10, 
                                                    current = q_img,
                                                    show = show, 
                                                    frame = frame, 
                                                    live = True, 
                                                    show_iterations = False)
        return new_center, current

    except Exception as inst: #printing the error associated with why the except code chunk ran
        print inst
        print "Likely there are no matches"
        return None, current

def mean_shift(hypothesis, keypoints, threshold, frame, current = None, show = False, live = False, show_iterations = False):
    """
    Inputs:
        hypothesis -> Previous center point as a starting hypothesis
        keypoints -> List of keypoint (x,y) coordinates
        Threshold -> maximum acceptable difference in center between iterations (eg 10 pixels, 5 pixels)
        current -> np array representing the image (for visualization)
        show -> determines whether visualization is shown

    Returns:
        New center of keypoints
        Radius
        If show is true -> displays the center, keypoints and a circle around the object
    """

    n=0
    if len(keypoints) > 5: # increased to 5 for testing purposes

        #assigns a value to the weighting constant -> based on 
        #experimental results on cropped cookie_00274
        c = 0.0001

        #arbitrarily set diff high to go through loop at least once
        diff = 1000

        while(diff > threshold):
            #sets up lists of weights and weights*position
            x_weights = []
            y_weights = []
            weighted_x = []
            weighted_y = []
            #Creats a list of weighted points, where points near the 
            #hypothesis have a larger weight
            last_guess = hypothesis
            for kp in keypoints:
                x_val = np.exp(-c * (kp[0] - last_guess[0])**2)
                x_weights.append(x_val)
                weighted_x.append(x_val*kp[0])
                y_val = np.exp(-c * (kp[1] - last_guess[1])**2)
                y_weights.append(y_val)
                weighted_y.append(y_val*kp[1])

            #finds 'center of mass' of the points to determine new center
            x = int(sum(weighted_x)/sum(x_weights))
            y = int(sum(weighted_y)/sum(y_weights))

            #update hypothesis
            hypothesis = (x,y)

            #difference between the current and last guess
            diff = np.sqrt((last_guess[0] - x)**2 + (last_guess[1] - y)**2)


            #visualizes moving center and displays keypoints for every meanshift iteration if show_iterations==True
            if show_iterations:
                img = np.copy(current)   #Needs to be np array (already opened by cv2)
                for k in keypoints:
                    cv2.circle(img, k, 2, [255, 0, 0], 2)
                cv2.circle(img, hypothesis, 3, [0, 0, 255], 3)
                cv2.circle(img, hypothesis, 5, [100,255,0], 2)
                cv2.imshow('Frame %d: Current hypothesis, meanshift guess %d' % (frame, n), img)
                # cv2.imwrite('../OT_res/meanshift_presentation/cookie_f%d_guess%d.jpg' % (frame, n), img)
                cv2.waitKey(0)
                n+=1
        
        #visualizes moving center and displays keypoints if show==True for the frame 
        #(so, this happens once per call of the function)
        if show:
            draw_circles(img=current, kp=keypoints, c = hypothesis, radius = 5)
            if not live:
                cv2.imshow('current', current)
                cv2.waitKey(0)

        return hypothesis, current

def play_wave(filename, player='aplay'):
    """plays an inputted wav file
    """
    cmd = '{} {}'.format(player, filename)
    popen = subprocess.Popen(cmd, shell=True)
    popen.communicate()

def play_audio(center, previous_file):
    """plays a generated audio file based on the inputted center
    inputs: center - the center of the kyepoints for a tracked image (x,y)
            previous_file - the previous audio file we played
    """
    w = 512 # TODO: put in width of image
    max_height = 512 # TODO: height of the image
    path = "../../GeneratedSoundFiles/"
    max_angle = np.pi / 2
    min_angle = -max_angle
    # angle = np.arctan(center[0])
    angle = (center[0]/640.0) * 170.0 - 85.0
    # height_bin = 7 - (int(round(center[1] / max_height * 7)))
    height_bin = 4
    angle_bin = int(myround(angle))
    print angle_bin
    if angle_bin % 10 == 0:
        filename = previous_file
    else:
        if angle_bin > 0:
            filename = "{}height{}angle{}.wav".format(path, height_bin, angle_bin)
        else:
            filename = "{}height{}angle_{}.wav".format(path, height_bin, abs(angle_bin))
    play_wave(filename, player="aplay")
    return filename

def myround(x, base=5):
    return int(base * round(float(x)/base))

def dictionify(array):
    """takes in an array and outputs a dictionary where the keys are the first value of each row"""
    dictionary = {}
    for row in array:
        dictionary[row[0]] = row[1:]
    return dictionary

if __name__ == '__main__':
    # initial values for prototyping w/ the cookies. :P
    old_center = [1000, 170]
    center = old_center
    pos = [744,514,606,392]

    # with open('../gstore_csv/cookie.csv', 'r') as csvfile:
    #     reader = csv.reader(csvfile)

    data = np.loadtxt('../../gstore_csv/cookie.csv', dtype=int, delimiter=',')
    data_dict = dictionify(data)

    it_works = 0

    start = 180
    stop = 190

    for frame in range(start, stop): 
        cv2.destroyAllWindows()
        print "Frame number: %d" % frame
        center, current = match_object(previous = center, 
                              current = '../gstore_snippets/cookie_snippet/cookie_00%d.jpg' % frame, 
                              train_img = '../gstore_snippets/cookie_snippet/cookie_00177.jpg',
                              pos = pos,
                              show = False,
                              frame = frame)

        old_center = center

        if data_dict[frame][2] <= old_center[0] <= data_dict[frame][0] and data_dict[frame][3] <= old_center[1] <= data_dict[frame][1]:
            it_works = it_works +1
        
    percent_success = float(it_works)/(stop-start)
    print(percent_success)



  