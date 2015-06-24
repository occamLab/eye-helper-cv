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
    def __init__(self, tracker, height=1.69, parts=None):
        """
        height is in meters, for now. we could convert if that's easier though.
        1.69 meters is average-ish for u.s. adult height.
        parts is a list or tuple of the height of the knee, waist, chest, chin, and eyes. It defaults to none, in which case it uses the normal proportions for people times the selecteds height. 
        """
        self.tracker = tracker
        self.isOn = False
        self.height = height
        if self.parts != None:
            self.parts = parts
        else:
            self.parts = 
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
