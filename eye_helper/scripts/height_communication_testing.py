#!/usr/bin/env python

"""

Trying out a couple different ways of encoding height information.

"""

import rospy
import rospkg
import subprocess
from tango_tracker import Tango_tracker
import string
from eye_helper.msg import Speech

class Absolute_height():
    """
    Communicates the absolute height to the object, from the reference of the tango.
    """
    def __init__(self, tracker, isMetric=False):
        self.tracker = tracker
        self.isOn = False
        self.isMetric = isMetric
        self.last_played = rospy.Time.now()
        self.player = "aplay"
        self.rospack = rospkg.RosPack()
        self.path = self.rospack.get_path('eye_helper') + "../GeneratedSoundFiles/wavs/wavs2/"
        self.base_filename = "{}.wav"
        self.filename = ""
        self.speech_pub=rospy.Publisher('/speech_info', Speech, queue_size=10)

    def turn_on(self):
        self.isOn = True

    def turn_off(self):
        self.isOn = False

    def toggle(self):
        self.isOn = not self.isOn

    def call(self):
        if self.isOn:
            self.tracker.refresh_all()
            if self.tracker.z_distance != None and self.tracker.right_distance != None and self.tracker.forward_distance != None:
                self.run()

    def run(self):
        zd = self.tracker.z_distance #in meters
        zd_inches=round(39.3701*zd,1) #in inches
        if abs(zd_inches) == 0:
            pass
        if abs(zd_inches)<0:
            u='down'
        if abs(zd_inches)>0:
            u='up'
        values_to_play.append(str(zd_inches)[0:str(zd_inches).index('.')])
        values_to_play.append('point')
        values_to_play.append(str(zd_inches)[str(zd_inches).index('.')+1:len(str(zd))])
        values_to_play.append('inches')
        values_to_play.append(u)

####################################################### PLAYING SOUND FILES ##########################################################################

        p = subprocess.Popen('amixer -D pulse sset Master 30%', shell=True)
        p.communicate()

        for i in values_to_play:
            print i
            self.filename = '{} {}{}.wav'.format(self.player, self.path, i)
            p = subprocess.Popen('{} {}{}.wav'.format(self.player, self.path, i), shell=True)
            p.communicate()

            self.speech_info= Speech(file_path=self.path + self.filename, speech=str(i))
            self.speech_pub.publish(self.speech_info)

class Angle_height():
    """
    Speaks the location of the object forward, right/left, and up/down from the tango.
    Note that this is probably only useful really close to the object. Further away, most of the info isn't all that handy.
    """
    def __init__(self, tracker):
        self.tracker = tracker
        self.isOn = False
        self.last_played = rospy.Time.now()
        self.player = "aplay"
        self.rospack = rospkg.RosPack();
        self.path = self.rospack.get_path('eye_helper') + "/../GeneratedSoundFiles/wavs/wavs2/"
        self.base_filename = "{}.wav"
        self.filename = ""
        self.speech_pub=rospy.Publisher('/speech_info', Speech, queue_size=10)

    def toggle(self):
        self.isOn = not self.isOn

    def turn_on(self):
        self.isOn = True

    def turn_off(self):
        self.isOn = False

    def call(self):
        if self.isOn:
            self.tracker.refresh_all()
            if self.tracker.z_distance != None and self.tracker.right_distance != None and self.tracker.forward_distance != None:
                self.run()


    def run(self):

        self.last_played = rospy.Time.now()
        zd_signed = self.tracker.z_distance
        zd = abs(zd_signed)
        height= self.tracker.pitch * 57.2957795
        values_to_play = []

# ============================================== UP - DOWN MAPPING =============================================================================================if atg<0:
            if height<-3:
                h='down'
            if height>3:
                h='up'
            if abs(height)> 3 and abs(height) <= 7.5:
                values_to_play.append('5'+h)
            if abs(height)> 7.5 and abs(height) <= 12.5:
                values_to_play.append('10'+h)
            if abs(height)> 12.5 and abs(height) <= 17.5:
                values_to_play.append('15'+h)
            if abs(height)> 17.5 and abs(height) <= 22.5:
                values_to_play.append('20'+h)
            if abs(height)>22.5 and abs(height) <= 27.5:
                values_to_play.append('25'+h)
            if abs(height)> 27.5 and abs(height) <= 32.5:
                values_to_play.append('30'+h)
            if abs(height)> 32.5 and abs(height) <= 37.5:
                values_to_play.append('35'+h)
            if abs(height)> 37.5 and abs(height) <= 42.5:
                values_to_play.append('40'+h)
            if abs(height)> 42.5 and abs(height) <= 47.5:
                values_to_play.append('45'+h)
            if abs(height)> 47.5 and abs(height) <= 52.5:
                values_to_play.append('50'+h)
            if abs(height)> 52.5 and abs(height) <= 57.5:
                values_to_play.append('55'+h)
            if abs(height)> 57.5 and abs(height) <= 62.5:
                values_to_play.append('60'+h)

# ============================================= PLAYING SOUND FILES TO SPEAK ===================================================================================
        p = subprocess.Popen('amixer -D pulse sset Master 30%', shell=True)
        p.communicate()

        for i in values_to_play:
            self.filename = '{} {}{}.wav'.format(self.player, self.path, i)
            p = subprocess.Popen('{} {}{}.wav'.format(self.player, self.path, i), shell=True)
            p.communicate()

            self.speech_info= Speech(file_path=self.path + self.filename, speech=str(i))
            self.speech_pub.publish(self.speech_info)



class Body_mapping():
    """
    Tries to approximate the corresponding __-level.
    """
    def __init__(self, tracker, height=1.68, parts=None, proportion=None, tango_height=1):
        """
        height is in meters, for now. we could convert if that's easier though.
        1.68 meters is average-ish for u.s. adult height.
        parts is a dict of the height of the knee, hip, elbow, shoulder, and eyes. It defaults to none, in which case it uses the normal proportions for people times the selecteds height. 
        Parts takes precedence over proportion, but only one's really useful at once.
        Tango height is what it sounds like.
        """
        self.tracker = tracker
        self.isOn = False

        self.height = height
        self.tango_height = tango_height
        if parts != None:
            self.parts = parts
        elif proportion != None:
            self.parts = {i: self.height*proportion[i] for i in proportion}
        else:
            proportion = {"eye": 0.938, "shoulder": 0.825, "elbow": 0.63, "hip": 0.522, "knee": 0.336}
            self.parts = {i: self.height*proportion[i] for i in proportion}

        self.last_played = rospy.Time.now()
        self.player = "aplay"
        self.rospack = rospkg.RosPack()
        self.path = self.rospack.get_path('eye_helper') + "../GeneratedSoundFiles/height_speech"

    def turn_on(self):
        self.isOn = True

    def turn_off(self):
        self.isOn = False

    def toggle(self):
        self.isOn = not self.isOn

    def call(self):
        if self.isOn:
            self.tracker.refresh_all()
            if self.tracker.z_distance != None:
                self.run()

    def run(self):
        target_h = self.tango_height + self.tracker.z_distance
        target_to_part_distance = {i: target_h - self.parts[i] for i in self.parts}
        closest_part = min(target_to_part_distance, key = abs(target_to_part_distance.get))
        d = target_to_part_distance[closest_part]
        #once we figure out what the sound files are, they would be played here.
        #I'm personally inclined to organizing them something like,
        # [elevation relative to] [part], e.g.:
        # "{} {}-level".format("at", "eye")
        # "{} {}-level".format("slightly below", "shoulder")
        # "{} {}-level".format("half a foot above", "knee")
        # etc.
