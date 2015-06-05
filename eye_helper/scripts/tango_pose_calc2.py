#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import math
import subprocess
import rospkg
# from cv_bridge import CvBridge, CvBridgeError
# from image_selector import *
# from object_matcher import *
# import audio_player
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PointStamped
import threading 
import time
from tf.transformations import euler_from_quaternion#, rotation_matrix, quaternion_from_matrix
from std_msgs.msg import Float64, Float64MultiArray, String
import math
from tango_tracker import Tango_tracker

class TangoPoseCalc():
    """
    Calculate relative locations w/ Tango
    """

    def __init__(self):

        self.tracker = Tango_tracker()
        self.logger = rospy.Publisher('/log', String, queue_size=10)
        self.forward_distance_pub = rospy.Publisher('/forward_distance', Float64, queue_size=10)
        self.right_distance_pub = rospy.Publisher('/right_distance', Float64, queue_size=10)

        self.rospack = rospkg.RosPack();
        self.path = self.rospack.get_path('eye_helper') + '/../GeneratedSoundFiles/'
        # Stuff for Tango coord use:

        self.player = 'aplay'
        self.playback_interval = rospy.Duration(0.2)
        self.last_tone = rospy.Time.now()

    def play_wave(self, volume, mix=None):
        """
        plays an inputted wav file
        """
        self.logger.publish(String(data="played audio file %s at volume %f" % (self.filename, volume)))
        cmd = 'amixer set Master {}'.format(volume)
        print cmd
        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()
        if mix != None:
            # need to scale this by volume as a percentage.... TODO
            cmd = 'amixer sset Headphone {}%,{}%'.format(mix[0],mix[1])
            popen = subprocess.Popen(cmd, shell=True)
            popen.communicate()
        # print self.filename
        cmd = '{} {}'.format(self.player, self.filename)
        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()


    def adjust_playback_interval(self,d):
        if d != None:
            self.playback_interval = rospy.Duration(d/2.0)


    def get_h_to_play(self, rz):
        """
        returns the h-value to play. Currently the numbers are more-or-less arbitrary; TODO: test/figure out what works well. rz = relative_z of the target compared to the tango.
        """
        if rz <= -0.6:
            return 1
        elif rz <= -0.2:
            return 2
        elif rz <= 0.2:
            return 3
        elif rz <=0.6:
            return 4
        else:
            return 6


    def play_audio(self):
        """
            Uses class variables; plays the corresponding sound file.

            Edit: Angle_to_volume part is commented out. Instead, new lines of code added for y_to_volume to adjust volume according to y-distance. -pdemetci
        """
        self.tracker.refresh_all()
        d = self.tracker.forward_distance
        eh.adjust_playback_interval(d)
        self.forward_distance_pub.publish(Float64(self.tracker.forward_distance))
        self.right_distance_pub.publish(Float64(self.tracker.right_distance))
        #print self.tracker.xy_distance
        print self.tracker.yaw, self.tracker.forward_distance, self.tracker.right_distance
        # print self.playback_interval
        if rospy.Time.now() - self.last_tone < self.playback_interval:
            return
        self.last_tone = rospy.Time.now()
        print "playing audio"

        if d == None:
            return
        # elif d >= 1:


        #     if self.target_z != None: # This check is probably now unnecessary.
        #         self.update_angle_to_go()
        #         self.angle_to_play = int(5 * round(float(self.angle_to_go)/5))
        #         angle_to_volume = {0: 21, 5: 22, 10: 23, 15: 24, 20: 25, 25: 26, 30: 27, 35: 28, 40: 29, 45: 30, 50: 31, 55:31, 60:31, 65:31, 70:31, 75 : 31, 80 : 31, 85 : 31, 90: 31}
        #         desired_volume = angle_to_volume[abs(self.angle_to_play)]
        #         # print self.angle_to_play
        #         if self.angle_to_play >= 0:
        #             self.filename = "{}height{}angle{}.wav".format(self.path, 4, 90)
        #             #self.filename = "{}height{}angle{}.wav".format(self.path, 4, self.angle_to_play)
        #         else:
        #             self.filename = "{}height{}angle_{}.wav".format(self.path, 4, 90)
        #             #                self.filename = "{}height{}angle_{}.wav".format(self.path, 4, abs(self.angle_to_play))
        #   
        else:
            #relative_z = self.target_z - self.z

            ###Code for left-right mix:

            if self.tracker.right_distance < 0:
                desired_mix = ['100','0']
            elif self.tracker.right_distance==0:
                desired_mix = ['50','50']
            else:
                desired_mix = ['0','100']

            ### Code for playing volume based on y-distance (instead of yaw)### 
            relative_y = abs(self.tracker.right_distance)
            if relative_y > 1.8:
                relative_y=2
            if relative_y > 1.6 and relative_y <=1.8:
                relative_y=1.8
            if relative_y >1.4 and relative_y <=1.6 :
                relative_y=1.6
            if relative_y > 1.2 and relative_y <=1.4:
                relative_y=1.4
            if relative_y > 1.0 and relative_y <=1.2:
                relative_y=1.2
            if relative_y > 0.8 and relative_y <=1.0:
                relative_y=1.0
            if relative_y > 0.6 and relative_y <=0.8:
                relative_y=0.8
            if relative_y > 0.4 and relative_y <=0.6:
                relative_y=0.6
            if relative_y > 0.2 and relative_y <=0.4:
                relative_y=0.4
            if relative_y<=0.2:
                relative_y=0.2

            possible_y=[0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0]
            possible_volume=[x for x in range(20,66) if x%5 == 0]   #Just because I did not have any audio files where volume>31. Can be commented out/deleted once we create such files 
            y_to_volume={}
            for i in range(len(possible_y)):
                y_to_volume[possible_y[i]]=possible_volume[i]

            desired_volume=y_to_volume[relative_y]

            self.filename="{}volume{}.wav".format(self.path, 22)
            desired_volume*=4
            print "desired volume", desired_volume
            self.play_wave(desired_volume, desired_mix)

            ### Code for angle_to_volume ###

            # angle_to_volume = {0: 21, 5: 22, 10: 23, 15: 24, 20: 25, 25: 26, 30: 27, 35: 28, 40: 29, 45: 30, 50: 31, 55:31, 60:31, 65:31, 70:31, 75 : 31, 80 : 31, 85 : 31, 90: 31}
            # desired_volume = angle_to_volume[abs(self.angle_to_play)]
            # h_to_play = self.get_h_to_play(relative_z)

            #     #                self.filename = "{}height{}angle_{}.wav".format(self.path, 4, abs(self.angle_to_play))
            # print "volume is", desired_volume
            # print "desired mix", desired_mix
            # #desired_mix = None
            


if __name__ == "__main__":
    eh = TangoPoseCalc()
    r = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        r.sleep()
        # eh.tango_get_angle_to_go()
        # Needs to get its targets chosen before this will work yet :P.
        eh.play_audio()
