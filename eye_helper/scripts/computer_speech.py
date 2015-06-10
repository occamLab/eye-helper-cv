"""
Modules/versions which provide information by playing computer speech files.

"""

import rospy
import subprocess
from tango_tracker import Tango_tracker
import string
#import Tkinter as tk

class Orthogonal_distances():
    """
    Forward and sideways distances. Not really sure how to communicate this, but anyways. Right now, it's just 'spoken' (computer speech).
    """
    def __init__(self, tracker):
        self.tracker = tracker
        self.isOn = False
        self.volume_coefficient = 1.0 #thing to change.
        self.delay_coefficient = 0.5 #thing to change.
        self.last_played = rospy.Time.now()
        self.player = "aplay"
        self.path = "../../GeneratedSoundFiles/computer_speech/"
        self.base_filename = "{}.wav"
        self.filename = ""

    def toggle(self):
        self.isOn = not self.isOn

    def call(self):
        if self.isOn:
            self.tracker.refresh_all()
            if self.tracker.forward_distance != None and self.tracker.right_distance != None:
                self.run()

    def run(self):
        if (rospy.Time.now() - self.last_played) < rospy.Duration(8):
            return
        self.last_played = rospy.Time.now()
        fd_signed = self.tracker.forward_distance
        fd = abs(fd_signed)
        rd_signed = self.tracker.right_distance
        rd = abs(rd_signed)
        values_to_play = [str(int(fd)), 'pt', str(int(10*((fd - int(fd)) - (fd - int(fd))%0.1)))]
        if fd_signed > 0:
            values_to_play.append('forward')
        else:
            values_to_play.append('back')
        values_to_play.append('and')
        values_to_play.extend([str(int(rd)),'pt',str(int(10*((rd-int(rd))-(rd-int(rd))%.1)))])
        if rd_signed > 0:
            values_to_play.append('right')
        else:
            values_to_play.append('left')
        for i in values_to_play:
            popen = subprocess.Popen('{} {}{}.wav'.format(self.player, self.path,i), shell=True)
            popen.communicate()

class Speak_3d_coords():
    """
    Speaks the location of the object forward, right/left, and up/down from the tango.
    Note that this is probably only useful really close to the object. Further away, most of the info isn't all that handy.
    """
    def __init__(self, tracker):
        self.tracker = tracker
        self.isOn = False
        self.last_played = rospy.Time.now()
        self.player = "aplay"
        self.path = "../../GeneratedSoundFiles/wavs/"
        self.base_filename = "{}.wav"
        self.filename = ""

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
        # if rospy.Time.now() - self.last_played < rospy.Duration(10):
            # return
        self.last_played = rospy.Time.now()
        fd_signed = self.tracker.forward_distance
        fd = abs(fd_signed)
        rd_signed = self.tracker.right_distance
        rd = abs(rd_signed)
        zd_signed = self.tracker.z_distance
        zd = abs(zd_signed)

        values_to_play = []
        cmds = []
        values_to_play.append(str(fd)[0]+'p'+str(fd)[2]+'m')
        if fd_signed >= 0:
            values_to_play.append('fa')
        else:
            values_to_play.append('ba')
        cmds.append("".join(values_to_play))
        values_to_play = []
        values_to_play.append(str(rd)[0]+'p'+str(rd)[2]+'m')
        if rd_signed >= 0:
            values_to_play.append('ra')
        else:
            values_to_play.append('la')
        cmds.append("".join(values_to_play))
        values_to_play = []

        values_to_play.append(str(zd)[0]+'p'+str(zd)[2]+'m')
        if zd_signed >= 0:
            values_to_play.append('u')
        else:
            values_to_play.append('d')
        cmds.append("".join(values_to_play))
        values_to_play = []

        p = subprocess.Popen('amixer -D pulse sset Master 30%', shell=True)
        p.communicate()

        for i in cmds:
            p = subprocess.Popen('{} {}{}.wav'.format(self.player, self.path, i), shell=True)
            p.communicate()



    # def run(self):
    #     if rospy.Time.now() - self.last_played < rospy.Duration(10):
    #         return
    #     self.last_played = rospy.Time.now()
    #     fd_signed = self.tracker.forward_distance
    #     fd = abs(fd_signed)
    #     rd_signed = self.tracker.right_distance
    #     rd = abs(rd_signed)
    #     z_signed = self.tracker.z_distance
    #     z = abs(z_signed)

    #     values_to_play = ["theObjectIs", str(int(fd)), 'pt', str(int(10*((fd - int(fd)) - (fd - int(fd))%0.1))), "meters"]
    #     if fd_signed > 0:
    #         values_to_play.append('forward')
    #     else:
    #         values_to_play.append('back')
    #     values_to_play.append('and')
    #     values_to_play.extend([str(int(rd)),'pt',str(int(10*((rd-int(rd))-(rd-int(rd))%.1))), "meters", "toYour"])
    #     if rd_signed > 0:
    #         values_to_play.append('right')
    #     else:
    #         values_to_play.append('left')
    #     values_to_play.append('and')
    #     values_to_play.extend([str(int(z)), 'pt', str(int(10*((z-int(z)) - (z - int(z))%.1))), "meters"])
    #     if z_signed > 0:
    #         values_to_play.append('above')
    #     else:
    #         values_to_play.append("below")
    #     values_to_play.append("theTango")
    #     popen = subprocess.Popen('amixer -D pulse sset Master 40%', shell=True)
    #     popen.communicate()
    #     for i in values_to_play:
    #         popen = subprocess.Popen("{} {}{}.wav".format(self.player, self.path, i), shell=True)
    #         popen.communicate()



