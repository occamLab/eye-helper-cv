#!/usr/bin/env python

import rospy
import rospkg
import subprocess
# from tango_tracker import Tango_tracker
from offset_tracker import Offset_tracker
import Tkinter as tk
from eye_helper.msg import Sound
from eye_helper.msg import Speech

class Angle_body_mapping():

    def __init__(self, tracker, height=1.65, parts=None, proportion={"eye": 0.938, "shoulder": 0.825, "elbow": 0.63, "hip": 0.522, "knee": 0.336}, tango_height=1.06):
        self.tracker = tracker
        self.isOn = False
#------------------------------------------------- For body mapping ------------------------------------
        self.height = height
        self.tango_height = tango_height
        self.proportions = proportion
        if parts != None:
            self.parts = parts
        else:
            self.parts = {i: self.height*proportion[i] for i in proportion}
#------------------------------------------------- For Angle_beep --------------------------------------
        self.reverse = False #thing to change - changes whether sound-in-right-ear means move *to*, or *away from* the right.
        self.volume_coefficient = 2 #thing to change.
        self.minimum_volume = 15 #thing to change.
        self.max_volume = 75 #thing to change.
        self.delay_coefficient = 0.5 #thing to change
#------------------------------------------------------ For playing -------------------------------------
        self.last_played= rospy.Time.now()
        self.last_tone = rospy.Time.now()
        self.player = "aplay"
        self.rospack = rospkg.RosPack();
        self.path = self.rospack.get_path('eye_helper') + "/../GeneratedSoundFiles/wavs/body_mapping/"
        self.filename =  None 
        self.sound_pub=rospy.Publisher('/sound_info', Sound, queue_size=10)
        self.speech_pub=rospy.Publisher('/speech_info', Speech, queue_size=10)
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
            if self.tracker.xy_distance != None and self.tracker.angle_to_target!= None:
                self.run()
        
    def run(self):
        if self.tracker.xy_distance>0.5 or self.ratio!=[1,1]:
            self.angle_beeps()
        else:
            self.body_mapping()

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


    def body_mapping(self):
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

        popen = subprocess.Popen('amixer -D pulse sset Master 30%', shell=True)
        popen.communicate()
        popen = subprocess.Popen("aplay {}{}".format(self.path, file_to_play), shell=True)
        popen.communicate()
        self.speech_info= Speech(file_path=self.path + file_to_play, speech=str(location)+str( )+str(direction))
        self.speech_pub.publish(self.speech_info)

    def angle_beeps(self):
        self.filename="height4angle5.wav"
        delay = rospy.Duration(self.dist_to_delay(self.delay_coefficient))
        if rospy.Time.now() - self.last_tone < delay:
            return
        self.last_tone = rospy.Time.now()
        vol = self.angle_to_volume(self.tracker, self.volume_coefficient)
        atg = self.tracker.angle_to_target
        if self.reverse:
            if atg >= 0:
                self.ratio = [0,1]
            else:
                self.ratio = [1,0]
        else:
            if atg >= 0:
                self.ratio=[0,1]
            else:
                self.ratio = [1,0]
        if abs(atg) * self.volume_coefficient < self.minimum_volume:
            vol = self.minimum_volume
            self.ratio = [1,1]
        elif abs(atg) * self.volume_coefficient > self.max_volume:
            vol = self.max_volume

        cmd = 'amixer -D pulse sset Master {}%,{}%'.format(vol*self.ratio[0], vol*self.ratio[1])
        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()
        cmd = "{} {}{}".format(self.player, self.path, self.filename)
        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()
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
        atg = tracker.angle_to_target
        v = abs(atg*coefficient)
        return v



if __name__ == "__main__":
    ot = Offset_tracker(offset_axes=[0,((0.45/2)-0.049),0],nodename="offset_tracker2", topic_name='/shoulder_offset')
    ot.refresh_all()
    feedback= Angle_body_mapping(ot)
    feedback.turn_on()
    while not rospy.is_shutdown():
        tt.refresh_all()
        feedback.call()
