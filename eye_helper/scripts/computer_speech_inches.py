#!/usr/bin/env python

"""
Modules/versions which provide information by playing computer speech files.
"""

import rospy
import rospkg
import subprocess
from tango_tracker import Tango_tracker
import string
from eye_helper.msg import Speech
import math

class Speak_3d_directions():
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
        fd_signed = self.tracker.forward_distance
        fd = abs(fd_signed)
        rd_signed = self.tracker.right_distance
        rd = abs(rd_signed)
        height= self.tracker.pitch * 57.2957795
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
            values_to_play.append('reach')
            if atg>=-3 and atg<=3 and height>=-3 and height<=3:
                values_to_play.append('forward')

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

# ============================================== UP - DOWN MAPPING =============================================================================================if atg<0:
            zd = self.tracker.z_distance #in meters
            zd_inches=round(39.3701*zd,1)
            if abs(zd_inches) == 0:
                pass
            if abs(zd_inches)<0:
                u='down'
            if abs(zd_inches)>0:
                u='up'
            values_to_play.append(str(zd)[0]+'point'+str(zd)[2]+'inches'+u)
            

# ============================================= PLAYING SOUND FILES TO SPEAK ===================================================================================
        p = subprocess.Popen('amixer -D pulse sset Master 30%', shell=True)
        p.communicate()

        for i in values_to_play:
            self.filename = '{} {}{}.wav'.format(self.player, self.path, i)
            p = subprocess.Popen('{} {}{}.wav'.format(self.player, self.path, i), shell=True)
            p.communicate()

            self.speech_info= Speech(file_path=self.path + self.filename, speech=str(i))
            self.speech_pub.publish(self.speech_info)

if __name__ == "__main__":
    tt = Tango_tracker()
    offset = Speak_3d_directions(tt)
    offset.turn_on()
    while not rospy.is_shutdown():
        offset.call()