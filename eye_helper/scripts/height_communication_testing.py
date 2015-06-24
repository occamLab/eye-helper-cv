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
        self.path = self.rospack.get_path('eye_helper') + "../GeneratedSoundFiles/height_speech"

    def turn_on(self):
        self.isOn = True

    def turn_off(self):
        self.isOn = False

    def toggle(self):
        self.isOn = not self.isOn

    def call(self):
        if self.isOn:
            if self.isMetric:
                self.run_metric()
            else:
                self.run_imperial()

    def run_metric(self):
        z = self.tracker.z_distance
        #once we have sound files, play them here.


    def run_imperial(self):
        z =  self.tracker.z_distance * 3.28084
        feet = int(z)
        inches = int((z - int(z))*12.0)
        #once we have sound files, play them here.

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