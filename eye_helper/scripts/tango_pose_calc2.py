#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import math
import subprocess
import rospkg
import collections

# from cv_bridge import CvBridge, CvBridgeError
# from image_selector import *
# from object_matcher import *
# import audio_player
from eye_helper.msg import Sound
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

        # New topics for Tango to publish sound info to:
        self.sound_pub=rospy.Publisher('/sound_info', Sound, queue_size=10)

        self.rospack = rospkg.RosPack();
        self.path = self.rospack.get_path('eye_helper') + '/../GeneratedSoundFiles/'
        # Stuff for Tango coord use:

        self.player = 'aplay'
        self.playback_interval = rospy.Duration(0.2)
        self.last_tone = rospy.Time.now()

        # All initialized as None but defined in play_audio():
        self.sound_info = None
    # def play_wave(self, volume, mix):
    #     """
    #     plays an inputted wav file
    #     """
    #     self.logger.publish(String(data="played audio file %s at volume %f" % (self.filename, volume)))
    #     cmd = 'amixer set Master {}'.format(volume)
    #     print cmd
    #     popen = subprocess.Popen(cmd, shell=True)
    #     popen.communicate()
    #     if mix != None:
    #         # need to scale this by volume as a percentage.... TODO
    #         cmd = 'amixer sset Headphone {}%,{}%'.format(mix[0],mix[1])
    #         popen = subprocess.Popen(cmd, shell=True)
    #         popen.communicate()
    #     # print self.filename
    #     cmd = '{} {}'.format(self.player, self.filename)
    #     popen = subprocess.Popen(cmd, shell=True)
    #     popen.communicate()


    def adjust_playback_interval(self,d):
        if d != None:
            self.playback_interval = rospy.Duration(d/2.0)

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
        print rospy.Time.now(), self.last_tone, self.playback_interval

        if rospy.Time.now() - self.last_tone < self.playback_interval:
            return
        self.last_tone = rospy.Time.now()
        print "playing audio"

        if d == None:
            return
            
        else:
            #relative_z = self.target_z - self.z

            ###Code for left-right mix:

            if self.tracker.right_distance < -0.03:
                desired_mix = ['100','0']
            elif self.tracker.right_distance<0.03 or self.tracker.right_distance> -0.03 :
                desired_mix = ['50','50']
            else: 
                desired_mix = ['0','100']

            ### Code for playing volume based on y-distance (instead of yaw)### 
            relative_y = abs(self.tracker.right_distance)
            if relative_y == 1.2 or relative_y > 1.2:
                relative_y = 1.2
            elif relative_y >= 1.1 or relative_y < 1.2: 
                relative_y=1.1
            elif relative_y >= 1.0 or relative_y < 1.1: 
                relative_y=1.0
            elif relative_y >= 0.9 or relative_y < 1.0: 
                relative_y=0.9
            elif relative_y >= 0.8 or relative_y < 0.9: 
                relative_y=0.8
            elif relative_y >= 0.7 or relative_y < 0.8: 
                relative_y=0.7
            elif relative_y >= 0.6 or relative_y < 0.7: 
                relative_y=0.6
            elif relative_y >= 0.5 or relative_y < 0.6: 
                relative_y=0.5
            elif relative_y >= 0.4 or relative_y < 0.5: 
                relative_y=0.4
            elif relative_y >= 0.3 or relative_y < 0.4: 
                relative_y=0.3
            elif relative_y >= 0.2 or relative_y < 0.3: 
                relative_y=0.2
            else:
                relative_y=0.0
            possible_y=[0.0, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2]
            possible_volume=[20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75]   #Just because I did not have any audio files where volume>31. Can be commented out/deleted once we create such files 
            y_to_volume={}
            for i in range(len(possible_y)):
                y_to_volume[possible_y[i]]=possible_volume[i]
            desired_volume=y_to_volume[relative_y]

            self.filename="{}height4angle15.wav".format(self.path)
            desired_volume*=4 
            print "desired volume", type(desired_volume)
            # self.play_wave(desired_volume, desired_mix)
            self.sound_info= Sound(file_path=self.filename,volume=float(desired_volume),mix_left=float(desired_mix[0]),mix_right=float(desired_mix[1]) )
            self.sound_pub.publish(self.sound_info)

if __name__ == "__main__":
    eh = TangoPoseCalc()
    r = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        r.sleep()
        # eh.tango_get_angle_to_go()
        # Needs to get its targets chosen before this will work yet :P.
        eh.play_audio()
