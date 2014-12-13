#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from image_selector import *
from object_matcher import *
from audio_player import *
from sensor_msgs.msg import Image
import threading
import time

class EyeHelper():
    """
    The wrapper to end all wrappers.
    """

    def __init__(self):
        # yaaay class variables
        self.center = ()
        self.state = 'no_grocery'

        self.ims = ImageSelector()       
        self.om = ObjectMatcher() 
        self.ap = AudioPlayer(self.om)

        # TODO: Figure out how to stop the thread when q is pushed
        threading.Thread(target=self.ap.audio_loop).start()
        # Camera via the comprobo raspberry pi setup
        rospy.init_node('eyehelper')
        rospy.Subscriber('/camera/image_raw', Image, self.process_frame)
        self.bridge = CvBridge()

    def process_frame(self, msg):
        # TODO: make sure the callback works
        """
        callback for camera stuff
        """
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.key = cv2.waitKey(5)
        ### continuous stuff happens here ###
        if self.state == 'grocery':
            # runs the object matching and displays the matches/center
            self.om.run(frame, self.ims.frozen_img, self.ims.t_img_corners)
            cv2.imshow('Running', frame)

            if self.key == ord('d'):
                self.state = 'no_grocery'
                self.om.center = None
            elif self.key == ord('q'):
                print 'exit'


        elif self.state == 'no_grocery':
            # continue showing the raw stream
            self.om.center = None
            cv2.imshow('Running', frame)
            if self.key == ord('s'):
                # just a transition state
                self.state = 'selecting'
                return
            elif self.key == ord('q'):
                # TODO: byebye
                threading.threa
                print 'byebye'
                return

        elif self.state == 'selecting':
            # ims.run will loop through selection until spacebar is hit and execution can then go as normal
            self.ims.run(frame)
            # after we finish selecting, we have a grocery, so change states
            self.state = 'grocery'
            self.om.center = (100, 100)

            return

if __name__ == "__main__":
    eh = EyeHelper()
    r = rospy.Rate(5) # 5hz

    while not rospy.is_shutdown():
        r.sleep()