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
import audio_player
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PointStamped
import threading 
import time
from tf.transformations import euler_from_quaternion#, rotation_matrix, quaternion_from_matrix
from std_msgs.msg import Float64, Float64MultiArray, String
import math

class TangoPoseCalc():
    """
    Calculate relative locations w/ Tango
    """

    def __init__(self):
        self.starting_x = None
        self.starting_y = None
        self.starting_z = None # Gets calibrated early; zeroes x, y, and z values.
        self.x = None
        self.y = None
        self.z = None # Gets overwritten ASAP.

        rospy.init_node('tangoposecalc')
        rospy.Subscriber('/tango_pose', PoseStamped, self.process_pose)
        rospy.Subscriber('/tango_angles', Float64MultiArray, self.process_angle)
        rospy.Subscriber('/clicked_point', PointStamped, self.set_target)
        self.logger = rospy.Publisher('/log', String, queue_size=10)
        self.rospack = rospkg.RosPack();
        self.path = self.rospack.get_path('eye_helper') + '/../GeneratedSoundFiles/'
        # Stuff for Tango coord use:

        self.target_x = None
        self.target_y = None
        self.target_z = None # Volunteer would write it; for tests, could set arbitrarily and then set up testing space for it.
        self.player = 'aplay'
        self.playback_interval = rospy.Duration(0.2)
        self.last_tone = rospy.Time.now()


    def process_pose(self, msg):
        """
        zeroes position data, then writes it to class varirables.
        """
        if self.starting_x == None:
            self.starting_x = msg.pose.position.x
            self.starting_y = msg.pose.position.y
            self.starting_z = msg.pose.position.z
        self.x = msg.pose.position.x - self.starting_x
        self.y = msg.pose.position.y - self.starting_y
        self.z = msg.pose.position.z - self.starting_z

    def get_distance_to_target(self):
        if self.x == None or self.target_x == None:
            return None
        return math.sqrt((self.x - self.target_x)**2 +
                         (self.y - self.target_y)**2 +
                         (self.z - self.target_z)**2)

    def process_angle(self, msg):
        """
        writes angle info to class variables.
        """
        self.yaw = msg.data[2]
        self.pitch = msg.data[1]
        self.roll = msg.data[0]
        print "processing angle"

    def set_target(self, msg):
        """
        writes the message info to the target.
        """
        self.target_x = msg.point.x - self.starting_x
        self.target_y = msg.point.y - self.starting_y
        self.target_z = msg.point.z - self.starting_z


    # def tango_get_angle_to_go(self):
    #     """
    #         takes self.theta (user's xy-angle) and self.need_to_go_y and x; uses atan2.
    #         Writes to self.need_to_go_angle (now in DEGREES).
    #     """
    #     self.angle_to_go = math.degrees(math.atan2(self.target_y - self.y, self.target_x - self.x) - self.theta)


    def play_wave(self, volume):
        """
        plays an inputted wav file
        """
        self.logger.publish(String(data="played audio file %s at volume %f" % (self.filename, volume)))
        cmd = 'amixer set Master {}'.format(volume)
        print cmd
        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()
        # print self.filename
        cmd = '{} {}'.format(self.player, self.filename)
        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()


    def adjust_playback_interval(self):
        d = self.get_distance_to_target()
        if d != None:
            self.playback_interval = rospy.Duration(d/2.0)

    def play_audio(self):
        """
            Uses class variables; plays the corresponding sound file.
            Right now, to test it out, it's just using the tango's absolute direction--rather than the direction to an as-of-yet-unknown target.
        """
        eh.adjust_playback_interval()
        print self.playback_interval
        if rospy.Time.now() - self.last_tone < self.playback_interval:
            return
        self.last_tone = rospy.Time.now()

        if self.target_z != None:
            max_angle = math.degrees(np.pi / 2)
            min_angle = -max_angle
            self.angle_to_go = math.degrees(math.atan2(self.target_y - self.y, self.target_x - self.x) - self.yaw)
            # print self.angle_to_go
            if self.angle_to_go >= max_angle:
                self.angle_to_go = max_angle
            elif self.angle_to_go <= min_angle:
                self.angle_to_go = min_angle
            self.angle_to_play = int(5 * round(float(self.angle_to_go)/5))
            angle_to_volume = {0: 21, 5: 22, 10: 23, 15: 24, 20: 25, 25: 26, 30: 27, 35: 28, 40: 29, 45: 30, 50: 31, 55:31, 60:31, 65:31, 70:31, 75 : 31, 80 : 31, 85 : 31, 90: 31}
            desired_volume = angle_to_volume[abs(self.angle_to_play)]
            # print self.angle_to_play
            if self.angle_to_play >= 0:
                self.filename = "{}height{}angle{}.wav".format(self.path, 4, 90)
                #self.filename = "{}height{}angle{}.wav".format(self.path, 4, self.angle_to_play)
            else:
                self.filename = "{}height{}angle_{}.wav".format(self.path, 4, 90)
                #                self.filename = "{}height{}angle_{}.wav".format(self.path, 4, abs(self.angle_to_play))
            self.play_wave(desired_volume)


if __name__ == "__main__":
    eh = TangoPoseCalc()
    r = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        r.sleep()
        # eh.tango_get_angle_to_go()
        # Needs to get its targets chosen before this will work yet :P.
        eh.play_audio()
