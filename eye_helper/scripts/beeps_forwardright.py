#!/usr/bin/env python

"""
Contains a "version 1" eye-helper, which connects distance to frequency and angle to volume/side.

"""
import rospy
import rospkg
import subprocess
from tango_tracker import Tango_tracker
from geometry_msgs.msg import PointStamped, Point
import math
import numpy as np
import time
from std_msgs.msg import Header
from tf import TransformListener
from eye_helper.msg import Sound

class Forward_right_angle_beep():
    """
    maps volume to amount that the angle is off, side to angle, and distance to frequency.
    """
    def __init__(self, tracker):
        self.tracker = tracker
        self.isOn = False
        self.reverse = True #thing to change - True means beep comes from the side where the object is. False is vica-versa. 
        self.volume_coefficient = 3 #thing to change.
        self.minimum_volume = 15 #thing to change.
        self.max_volume = 40 #thing to change.
        self.delay_coefficient = 0.5 #thing to change
        self.last_tone = rospy.Time.now()
        self.player = "aplay"
        self.rospack = rospkg.RosPack();
        self.path = self.rospack.get_path('eye_helper') + '/../GeneratedSoundFiles/'
        self.filename = "height4angle5.wav"
        self.sound_pub=rospy.Publisher('/sound_info', Sound, queue_size=10)
        self.fd=None #forward distance to the target
        self.rd=None #right distance to the target
        self.angle=None #engle to the target

    def toggle(self):
        self.isOn = not self.isOn

    def turn_on(self):
        self.isOn = True

    def turn_off(self):
        self.isOn = False

    def call(self):
        if self.isOn:
            self.tracker.refresh_all()
            if self.tracker.xy_distance != None and self.tracker.angle_to_go != None:
                self.run()

    def run(self):

        #-------------------------------------------- Angle Calculation --------------------------------------------

        self.fd= self.tracker.forward_distance
        self.rd= self.tracker.right_distance
        self.angle= math.atan(self.rd/self.fd) * 57.2957795 # multiplied so that it is in degrees just like atg etc. for consistency

        # In order to deal with over-correction problems

        if 90 < self.angle <= 180:
            self.angle = 90
        elif 180 < self.angle <= 270:
            self.angle = -90
        elif 270 < self.angle <= 360:
            self.angle = 360 - self.angle
        elif -360 < self.angle <= -270:
            self.angle = self.angle + 360
        elif -270 < self.angle <= -180:
            self.angle = 90
        elif -180 < self.angle <= -90:
            self.angle = -90

        #-----------------------------------------------------------------------------------------------------------

        delay = rospy.Duration(self.dist_to_delay(self.tracker, self.delay_coefficient))
        if rospy.Time.now() - self.last_tone < delay:
            return

        self.last_tone = rospy.Time.now()
        vol = self.angle_to_volume(self.tracker, self.volume_coefficient)

        if self.reverse:
            if self.angle >= 0:
                ratio = [0,1]
            else:
                ratio = [1,0]
        else:
            if self.angle >= 0:
                ratio=[1,0]
            else:
                ratio = [0,1]

        if abs(self.angle) * self.volume_coefficient < self.minimum_volume:
            vol = self.minimum_volume
            ratio = [1,1]

        elif abs(self.angle) * self.volume_coefficient > self.max_volume:
            vol = self.max_volume

        self.play_audio(vol, ratio)

        self.sound_info= Sound(file_path=self.path + self.filename,volume=float(vol),mix_left=float(ratio[0]),mix_right=float(ratio[1]) )
        self.sound_pub.publish(self.sound_info)


    def dist_to_delay(self, tracker, coefficient, cutoff=4, reverse=False):
        """input the tracker and the coefficient. Takes the xy distance from the tracker, multiplies it by the coefficient, and returns the value - basically, the time to wait. For example, 2m -> 1m with coeff = 2 means 4s -> 2s delay."""

        xy_distance = tracker.xy_distance
        if xy_distance > cutoff:
            xy_distance = cutoff

        if not reverse:
            return xy_distance*coefficient
        else:
            return (cutoff * coefficient) - (xy_distance * coefficient)

    def angle_to_volume(self, tracker, coefficient):
        v = abs(self.angle*coefficient)
        return v


    def play_audio(self, volume, ratio):
        cmd = 'amixer -D pulse sset Master {}%,{}%'.format(volume*ratio[0], volume*ratio[1])
        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()
        cmd = "{} {}{}".format(self.player, self.path, self.filename)
        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()

if __name__ == "__main__":
    tt = Tango_tracker()
    beep = Forward_right_angle_beep(tt)
    beep.turn_on()
    while not rospy.is_shutdown():
        beep.call()

#todo: look into hough line transform
