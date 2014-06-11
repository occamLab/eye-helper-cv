# -*- coding: utf-8 -*-
"""
Created on Wed Jun 11 14:34:59 2014

@author: koenigin and greenteawarrior
"""

import numpy as np
import cv2

#using the laptop's built-in camera for now
cap = cv2.VideoCapture(0)

while(True):
    #get the image
    im = cap.read()[1]
    #converts image to grayscale
    imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    #return, threshold 
    #look at image and determines threshold value
    ret,thresh = cv2.threshold(imgray,127,255,0)
    #based on the threshold value, find contours and puts them in an hierarchy from starkest 
    #difference to more similar things
    #can either use CHAIN_APPROX_SIMPLE or CHAIN_APPROX_NONE
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    #draw the contours on image
    cv2.drawContours(im, contours, -1, (0,255,0), 3)
    #show the image + drawn contours
    cv2.imshow("window title", im)
    if cv2.waitKey(1) & 0xFF == ord('q'): #hit the q key to end the loop
        break

#when everything's done, release the capture
cap.release()
cv2.destroyAllWindows()
