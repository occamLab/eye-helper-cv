#!/usr/bin/env python

"""
Version which gives angle to reach at and body-mapped height to reach.
"""

import rospy
import rospkg
import subprocess
from tango_tracker import Tango_tracker
import string
from eye_helper.msg import Speech
import math
import Tkinter as tk
from height_communication_testing import Body_map_controller

class Reach():
    def __init__(self, tracker, height=1.68, parts=None, proportion={"eye": 0.938, "shoulder": 0.825, "elbow": 0.63, "hip": 0.522, "knee": 0.336}, tango_height=1.02):
        """
        height is in meters, for now. we could convert if that's easier though.
        1.68 meters is average-ish for u.s. adult height.
        parts is a dict of the height of the knee, hip, elbow, shoulder, and eyes. It defaults to none, in which case it uses the normal proportions for people times the selecteds height. 
        Parts takes precedence over proportion, but only one's really useful at a time.
        Tango height is height above floor plane.
        """

        #====================for both=========================
        self.tracker = tracker
        self.isOn = False
        self.last_played = rospy.Time.now()
        self.player = "aplay"
        self.rospack = rospkg.RosPack()
        self.delay = rospy.Duration(3) #arbitrary

        #================height specifics====================
        self.height = height
        self.tango_height = tango_height
        self.proportions = proportion
        if parts != None:
            self.parts = parts
        else:
            self.parts = {i: self.height*proportion[i] for i in proportion}
        self.body_path = self.rospack.get_path('eye_helper') + "/../GeneratedSoundFiles/wavs/body_mapping/"


        #=============angle specifics============================

        self.angle_path = self.rospack.get_path('eye_helper') + "/../GeneratedSoundFiles/wavs/wavs2/"
        self.speech_pub = rospy.Publisher('/speech_info', Speech, queue_size=10)

    def toggle(self):
        self.isOn = not self.isOn

    def turn_on(self):
        self.isOn = True

    def turn_off(self):
        self.isOn = False

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


    def call(self):
        if self.isOn:
            self.tracker.refresh_all()
            if self.tracker.z_distance != None and self.tracker.right_distance != None and self.tracker.forward_distance != None:
                self.run()

    def run(self):
        if (rospy.Time.now() - self.last_played) < self.delay:
            return
        self.last_played = rospy.Time.now()
        self.run_height()
        self.run_angle()

    def run_height(self):

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
        self.play_audio(self.body_path, file_to_play)


    def run_angle(self):
        fd_signed = self.tracker.forward_distance
        fd = abs(fd_signed)
        rd_signed = self.tracker.right_distance
        rd = abs(rd_signed)
        zd_signed = self.tracker.z_distance
        zd = abs(zd_signed)
        height_signed = self.tracker.pitch * 57.2957795
        height= abs(height_signed)
        atg = self.tracker.angle_to_go
        values_to_play = []
        cmds = []
        

#==================================================== FORWARD - BACK MAPPING ================================================================================

        if fd_signed> 0.7 and rd_signed> 0.4:
            values_to_play.append('forward_right')
        elif fd_signed> 0.7 and rd_signed< -0.4:
            values_to_play.append('forward_left')
        elif fd_signed> 0.7 and rd < 0.4:
            values_to_play.append('forward')
        elif fd < 0.7 and rd_signed > 0.4:
            values_to_play.append('right')
        elif fd < 0.7 and rd_signed < -0.4:
            values_to_play.append('left')
        elif fd_signed <= 0.6 and rd<= 0.4:
            if atg>-3 and atg<3:
                values_to_play.append('reach_forward')

        # ============================================== RIGHT - LEFT MAPPING =========================================================================================
            if atg<0:
                s='right'
            if atg>0:
                s='left'
            if abs(atg)> 3 and abs(atg) <= 7.5:
                values_to_play.append('5'+s)
            if abs(atg)> 7.5 and abs(atg) <= 12.5:
                values_to_play.append('10'+s)
            if abs(atg)> 12.5 and abs(atg) <= 17.5:
                values_to_play.append('15'+s)
            if abs(atg)> 17.5 and abs(atg) <= 22.5:
                values_to_play.append('20'+s)
            if abs(atg)>22.5 and abs(atg) <= 27.5:
                values_to_play.append('25'+s)
            if abs(atg)> 27.5 and abs(atg) <= 32.5:
                values_to_play.append('30'+s)
            if abs(atg)> 32.5 and abs(atg) <= 37.5:
                values_to_play.append('35'+s)
            if abs(atg)> 37.5 and abs(atg) <= 42.5:
                values_to_play.append('40'+s)
            if abs(atg)> 42.5 and abs(atg) <= 47.5:
                values_to_play.append('45'+s)

        # ============================================== UP - DOWN MAPPING =============================================================================================


        # ============================================= PLAYING SOUND FILES TO SPEAK ===================================================================================
        cmds.append("".join(values_to_play))

        p = subprocess.Popen('amixer -D pulse sset Master 30%', shell=True)
        p.communicate()

        for i in cmds:
            print "=============== \n", i, "\n ==============="
            self.filename = '{} {}{}.wav'.format(self.player, self.angle_path, i)
            p = subprocess.Popen('{} {}{}.wav'.format(self.player, self.angle_path, i), shell=True)
            p.communicate()

        self.speech_info= Speech(file_path=self.angle_path + self.filename, speech=str(i))
        self.speech_pub.publish(self.speech_info)


    def play_audio(self, path, file_to_play):
        popen = subprocess.Popen('amixer -D pulse sset Master 30%', shell=True)
        popen.communicate()

        popen = subprocess.Popen("{} {}{}".format(self.player, path, file_to_play), shell=True)
        popen.communicate()


if __name__ == "__main__":
    tt = Tango_tracker()
    eh = Reach(tt, tango_height=1.02)
    eh.turn_on()
    control = Body_map_controller(eh)
    control.master.title("body map height setting")
    control.after(100, control.call_module)
    control.mainloop()