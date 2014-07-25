import cv2
import numpy as np 
import scipy as sp 
import time

def select_train(video, path = None, start_frame = None, end_frame = None, from_cam = True):
    """
    Work in progress. 

    Takes in a video/camera id and allows the user to freeze
    on a frame and select an object that they wish to track through later frames

    inputs: a camera number or a video name, path, start and end frame, and the input False
    returns:

    """
    #opens video, taking frames from a webcam if that is the source
    if from_cam:
        vid = cv2.captureFromCAM(video)
        while(t<20):
            frame = cv2.imread(vid)
            cv2.imshow('current frame', frame)
            #waits for spacekey to be pushed
            k = cv2.waitKey(5)
            if k == 32:
                break
            t +=1
        cv.imshow('Selection window')
        cv.waitKey(0)
    #If the video comes from a file, open file to read video
    else:
        for f in range(start_frame, end_frame+1):
            frame = cv2.imread(path+video)