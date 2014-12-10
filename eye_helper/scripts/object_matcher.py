#!/usr/bin/env python

import cv2
import numpy as np

class ObjectMatcher():
    def __init__(self, t_img, t_img_corners):
        self.t_img = t_img
        self.t_img_corners = t_img_corners
        self.t_kp = []
        self.t_d = []
        self.q_kp = []
        self.q_d = []
        self.q_img = []

        self.matcher = cv2.BFMatcher()
        self.matches = []

    def find_kpd(self, frame):
        # get t_img keypoints and descriptors if we don't have them yet
        #   only store the keypoints we care about (i.e. things within corners)
        # get the current frame's keypoints and descriptors
        pass 

    def matching(self):
        # matching happens
        # nearest neighbor tests
        pass

    def mean_shift(self):
        # find "center of mass" of the keypoints
        # also, edit eyehelper.center 
        pass 
