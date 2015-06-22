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
        print 'self is on'

    def turn_on(self):
        self.isOn = True
        print 'self is on'

    def turn_off(self):
        self.isOn = False

    def call(self):
        if self.isOn:
            self.tracker.refresh_all()
            if self.tracker.z_distance != None and self.tracker.right_distance != None and self.tracker.forward_distance != None:
                self.run()


    def run(self):
        # if rospy.Time.now() - self.last_played < rospy.Duration(10):
            # return
        print 'running'
        self.last_played = rospy.Time.now()
        fd_signed = self.tracker.forward_distance
        fd = abs(fd_signed)
        rd_signed = self.tracker.right_distance
        rd = abs(rd_signed)
        zd_signed = self.tracker.z_distance
        zd = abs(zd_signed)

        values_to_play = []
        cmds = []

        atg = self.tracker.angle_to_go
        print atg


        if fd_signed> 0.7 and rd_signed> 0.4:
            values_to_play.append('forward_right')
        elif fd_signed> 0.7 and rd_signed< -0.4:
            values_to_play.append('forward_left')
        elif fd_signed> 0.7 and rd < 0.4:
            values_to_play.append('forward')
        # elif fd_signed<0.0:
        #     values_to_play.append('turn_back')
        elif fd < 0.7 and rd_signed > 0.4:
            values_to_play.append('right')
        elif fd < 0.7 and rd_signed < -0.4:
            values_to_play.append('left')
        elif fd_signed <= 0.6 and rd<= 0.4:
            if atg>-8 and atg<8:
                values_to_play.append('reach_forward')
            if atg>=8 and atg<15:
                values_to_play.append('10left')
            if atg>=15 and atg<25.:
                values_to_play.append('20left')
            if atg>=25 and atg<35:
                values_to_play.append('30left')
            if atg>=35 and atg<45:
                values_to_play.append('40left')
            # if atg>=45 and atg<55:
            #     values_to_play.append('50left')
            # if atg>=55 and atg<65:
            #     values_to_play.append('60left')
            if atg<=-8 and atg>-15:
                values_to_play.append('10right')
            if atg<=-15 and atg>-25.:
                values_to_play.append('20right')
            if atg<=-25 and atg>-35:
                values_to_play.append('30right')
            if atg<=-35 and atg>-45:
                values_to_play.append('40right')
            # if atg<=-45 and atg>-55:
            #     values_to_play.append('50right')
            # if atg<=-55 and atg>-65:
            #     values_to_play.append('60right')




        cmds.append("".join(values_to_play))

        p = subprocess.Popen('amixer -D pulse sset Master 30%', shell=True)
        p.communicate()

        for i in cmds:
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