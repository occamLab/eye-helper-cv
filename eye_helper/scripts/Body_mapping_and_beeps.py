#!/usr/bin/env python

import rospy
import rospkg
import subprocess
from tango_tracker import Tango_tracker
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
import math
import numpy as np
import time
from std_msgs.msg import Header
from tf import TransformListener
from eye_helper.msg import Sound
import ransac
import string
from eye_helper.msg import Speech
import Tkinter as tk


class Angle_and_distance():
    """
    maps volume to amount that the angle is off, side to angle, and distance to frequency.
    """
    def __init__(self, tracker):
        self.tracker = tracker
        self.isOn = False
        self.reverse = False #thing to change - changes whether sound-in-right-ear means move *to*, or *away from* the right.
        self.volume_coefficient = 2 #thing to change.
        self.minimum_volume = 15 #thing to change.
        self.max_volume = 80 #thing to change.
        self.delay_coefficient = 0.5 #thing to change
        self.last_tone = rospy.Time.now()
        self.player = "aplay"
        self.rospack = rospkg.RosPack();
        self.path = self.rospack.get_path('eye_helper') + '/../GeneratedSoundFiles/'
        self.filename = "height4angle5.wav"
        self.sound_pub=rospy.Publisher('/sound_info', Sound, queue_size=10)
        self.ratio=None

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
        delay = rospy.Duration(self.dist_to_delay(self.delay_coefficient))
        if rospy.Time.now() - self.last_tone < delay:
            return
        self.last_tone = rospy.Time.now()
        vol = self.angle_to_volume(self.tracker, self.volume_coefficient)
        atg = self.tracker.angle_to_go
        if self.reverse:
            if atg >= 0:
                self.ratio = [0,1]
            else:
                self.ratio = [1,0]
        else:
            if atg >= 0:
                self.ratio=[1,0]
            else:
                self.ratio = [0,1]
        if abs(atg) * self.volume_coefficient < self.minimum_volume:
            vol = self.minimum_volume
            self.ratio = [1,1]
        elif abs(atg) * self.volume_coefficient > self.max_volume:
            vol = self.max_volume

        self.play_audio(vol)

        self.sound_info= Sound(file_path=self.path + self.filename,volume=float(vol),mix_left=float(self.ratio[0]),mix_right=float(self.ratio[1]) )
        self.sound_pub.publish(self.sound_info)


    def dist_to_delay(self, coefficient, cutoff=4, reverse=False):
        """input the tracker and the coefficient. Takes the xy distance from the tracker, multiplies it by the coefficient, and returns the value - basically, the time to wait. For example, 2m -> 1m with coeff = 2 means 4s -> 2s delay."""
        xy_distance = self.tracker.xy_distance
        if xy_distance > cutoff:
            xy_distance = cutoff
        if not reverse:
            return xy_distance*coefficient
        else:
            return (cutoff * coefficient) - (xy_distance * coefficient)

    def angle_to_volume(self, tracker, coefficient):
        atg = tracker.angle_to_go
        v = abs(atg*coefficient)
        return v


    def play_audio(self, volume):
        cmd = 'amixer -D pulse sset Master {}%,{}%'.format(volume*self.ratio[0], volume*self.ratio[1])
        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()
        cmd = "{} {}{}".format(self.player, self.path, self.filename)
        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()

class Body_mapping():
    """
    Tries to approximate the corresponding __-level.
    """
    def __init__(self, tracker, height=1.65, parts=None, proportion={"eye": 0.938, "shoulder": 0.825, "elbow": 0.63, "hip": 0.522, "knee": 0.336}, tango_height=1.06):
        """
        height is in meters, for now. we could convert if that's easier though.
        1.68 meters is average-ish for u.s. adult height.
        parts is a dict of the height of the knee, hip, elbow, shoulder, and eyes. It defaults to none, in which case it uses the normal proportions for people times the selecteds height. 
        Parts takes precedence over proportion, but only one's really useful at a time.
        Tango height is what it sounds like.
        """
        self.tracker = tracker
        self.isOn = False

        self.height = height
        self.tango_height = tango_height
        self.proportions = proportion
        if parts != None:
            self.parts = parts
        else:
            self.parts = {i: self.height*proportion[i] for i in proportion}

        self.last_played = rospy.Time.now()
        self.player = "aplay"
        self.rospack = rospkg.RosPack()
        self.path = self.rospack.get_path('eye_helper') + "/../GeneratedSoundFiles/wavs/body_mapping/"

    def turn_on(self):
        self.isOn = True
        self.run()

    def turn_off(self):
        self.isOn = False

    def toggle(self):
        self.isOn = not self.isOn

    def call(self):
        if self.isOn:
            self.tracker.refresh_all()
            if self.tracker.z_distance != None:
                self.run()

    def set_part(self, part, value):
        """
        overrides / replaces whatever the current height is for the input part.
        """
        if part in self.parts:
            self.parts[part] = value
            print part, " set at ", value

        else:
            print "Please use only eye, shoulder, elbow, hip, or knee."

    def set_tango_height(self, value):
        self.tango_height = value

    def set_person_height(self, value):
        self.height = value
        self.parts = {i: self.height*self.proportions[i] for i in self.proportions}


    def run(self):
        if rospy.Time.now() - self.last_played < rospy.Duration(4):
            return
        self.last_played = rospy.Time.now()
        target_h = self.tango_height + self.tracker.z_distance
        target_to_part_distance = {i: target_h - self.parts[i] for i in self.parts}
        keys = target_to_part_distance.keys()
        keys.sort(key = lambda x: abs(target_to_part_distance[x]))
        closest_part = keys[0]
        d = target_to_part_distance[closest_part]
        if d > 0.:
            direction = 'a'
        else:
            direction = 'b'
        feet = abs(d)/.3048
        if feet < 0.1:
            location = "at"
        elif feet < 0.4:
            location = "s" + direction
        elif feet < 0.9:
            location = "h" + direction
        else:
            location = "f" + direction
        file_to_play = "{}_{}.wav".format(location, closest_part[:3])

        self.play_audio(file_to_play)

    def play_audio(self, file_to_play):
        popen = subprocess.Popen('amixer -D pulse sset Master 30%', shell=True)
        popen.communicate()

        popen = subprocess.Popen("aplay {}{}".format(self.path, file_to_play), shell=True)
        popen.communicate()

if __name__ == "__main__":
    tt = Tango_tracker()
    body_map = Body_mapping(tt, tango_height=1.06)
    beeps=Angle_and_distance(tt)
    beeps.turn_on()
    while not rospy.is_shutdown():
        beeps.call()
        if tt.forward_distance<=0.6 and beeps.ratio==[1,1]:
            beeps.turn_off()
            body_map.turn_on()
            while not rospy.is_shutdown():
                body_map.call()
