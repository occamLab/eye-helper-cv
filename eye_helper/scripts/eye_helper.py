#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from image_selector import *
from object_matcher import *
from audio_player import *


class EyeHelper():
    """
    The wrapper to end all wrappers.
    """

    def __init__(self):
        # yaaay class variables
        self.center = []
        self.state = 'no_grocery'

        # initialize helper objects, yo 
        self.ims = ImageSelector()       
        self.om = ObjectMatcher()
        self.ap = AudioPlayer()

        # Camera via the comprobo raspberry pi setup
        rospy.init_node('eyehelper')
        rospy.Subscriber('/camera/image_raw', Image, process_frame)
        bridge = CvBridge()

    def process_frame(self, msg):
        # TODO: make sure the callback works
        "callback for camera stuff"
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        ### continuous stuff happens here ###

if __name__ == main:
    eh = EyeHelper()
    eh.run()

