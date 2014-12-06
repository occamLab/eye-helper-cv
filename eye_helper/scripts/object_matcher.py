#!/usr/bin/env python

import cv2
import numpy as np

class ObjectMatcher():
    def __init__(self, t_img):
        self.t_img = t_img
        self.t_kp = []
        self.t_d = []
        self.q_kp = []
        self.q_d = []
        self.q_img = []
        self.matches = []