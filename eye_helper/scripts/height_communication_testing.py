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
import math
import Tkinter as tk

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
        self.path = self.rospack.get_path('eye_helper') + "/../GeneratedSoundFiles/wavs/wavs2/"
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
            # if self.tracker.z_distance != None and self.tracker.right_distance != None and self.tracker.forward_distance != None:
            self.run()

    def run(self):
        values_to_play=[]
        zd = self.tracker.z_distance #in meters
        zd_inches=round(39.3701*zd,1) #converting to inches
        atg = self.tracker.angle_to_go

    # ============================================== RIGHT - LEFT MAPPING =========================================================================================
        if atg<0:
            s='right'
        if atg>0:
            s='left'
        if abs(atg)> 3 and abs(atg) <= 7.5:
            values_to_play.append('5'+s)
        else:
            rounded_atg = int(5 * round(float(atg)/5))
            values_to_play.append(str(rounded_atg)+s)
    # ============================================== UP - DOWN MAPPING =========================================================================================
        if abs(zd_inches) == 0 and abs(atg)==0:
            values_to_play.append('reach_forward')
        if abs(zd_inches)<0:
            u='down'
        if abs(zd_inches)>0:
            u='up'

        values_to_play.append(str(zd_inches)[0:str(zd_inches).index('.')]+'point')
        values_to_play.append(str(zd_inches)[str(zd_inches).index('.')+1:len(str(zd))]+'inches'+u)
        # values_to_play.append('inches')
        # values_to_play.append(u)

        #----------PLAYING SOUND FILES------------------------------------------------
        #-----------------------------------------------------------------------------
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
        self.delay=rospy.Duration(4)

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

        if rospy.Time.now() - self.last_played < self.delay:
            return
        
        zd_signed = self.tracker.z_distance
        zd = abs(zd_signed)
        atg = self.tracker.angle_to_go
        height= math.degrees(math.atan2(self.tracker.z_distance, self.tracker.xy_distance))
        values_to_play = []

    # ============================================== RIGHT - LEFT MAPPING =========================================================================================
        if atg<0:
            s='right'
        if atg>0:
            s='left'
        if abs(atg)> 3 and abs(atg) <= 7.5:
            values_to_play.append('5'+s)
        else:
            rounded_atg = int(5 * round(float(atg)/5))
            values_to_play.append(str(rounded_atg)+s)
    # ============================================== UP - DOWN MAPPING =============================================================================================if atg<0:
        if abs(height) > 3:
            if height<-3:
                h='down'
            else:
                h='up'
            rounded_angle = int(5 * round(float(height)/5)) # to the nearest 5.
            values_to_play.append(str(rounded_angle) + h)
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
    def __init__(self, tracker, height=1.68, parts=None, proportion={"eye": 0.938, "shoulder": 0.825, "elbow": 0.63, "hip": 0.522, "knee": 0.336}, tango_height=1):
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

    def set_height(self, value):
        self.tango_height = value

    def run(self):
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



class Body_map_controller(tk.Frame):
    def __init__(self, module, master=None):
        tk.Frame.__init__(self, master)
        self.module = module
        self.grid()
        self.createWidgets()


    def createWidgets(self):
        #eye, shoulder, elbow, hip, knee
        self.height_entry = tk.Entry(self)
        self.height_entry.grid(column=0, row=0)
        self.eye_entry = tk.Entry(self)
        self.eye_entry.grid(column=0, row=1)
        self.shoulder_entry = tk.Entry(self)
        self.shoulder_entry.grid(column=0, row=2)
        self.elbow_entry = tk.Entry(self)
        self.elbow_entry.grid(column=0, row=3)
        self.hip_entry = tk.Entry(self)
        self.hip_entry.grid(column=0, row=4)
        self.knee_entry = tk.Entry(self)
        self.knee_entry.grid(column=0, row=5)

        self.height_send = tk.Button(self, text = "send tango height", command = lambda: self.module.set_height(float(self.height_entry.get())))
        self.height_send.grid(column=1, row=0)
        self.eye_send = tk.Button(self, text = "send eye height", command = lambda: self.module.set_part("eye", float(self.eye_entry.get())))
        self.eye_send.grid(column=1, row=1)
        self.shoulder_send = tk.Button(self, text = "send shoulder height", command = lambda: self.module.set_part("shoulder", float(self.shoulder_entry.get())))
        self.shoulder_send.grid(column=1, row=2)
        self.elbow_send = tk.Button(self, text = "send elbow height", command = lambda: self.module.set_part("elbow", float(self.elbow_entry.get())))
        self.elbow_send.grid(column=1, row=3)
        self.hip_send = tk.Button(self, text = "send hip height", command = lambda: self.module.set_part("hip", float(self.hip_entry.get())))
        self.hip_send.grid(column=1, row=4)
        self.knee_send = tk.Button(self, text = "send knee height", command = lambda: self.module.set_part("knee", float(self.knee_entry.get())))
        self.knee_send.grid(column=1, row=5)

    def call_module(self):
        self.module.call()
        self.after(10, self.call_module)

if __name__ == "__main__":
    tt = Tango_tracker()
    body_map = Body_mapping(tt, tango_height=0.5)
    body_map = Body_mapping(None)
    body_map.turn_on()
    control = Body_map_controller(body_map)
    control.master.title("body map height setting")
    control.after(100, control.call_module)
    control.mainloop()