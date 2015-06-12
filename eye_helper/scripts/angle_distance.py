"""
Contains a "version 1" eye-helper, which connects distance to frequency and angle to volume/side.

"""
import rospy
import subprocess
from tango_tracker import Tango_tracker
from geometry_msgs.msg import PointStamped, Point
import math
import numpy as np
import time
from std_msgs.msg import Header
#import Tkinter as tk

class Angle_and_distance():
    """
    maps volume to amount that the angle is off, side to angle, and distance to frequency.
    """
    def __init__(self, tracker):
        self.tracker = tracker
        self.isOn = False
        self.volume_coefficient = 1.0 #thing to change.
        self.delay_coefficient = 0.5 #thing to change.
        self.last_tone = rospy.Time.now()
        self.player = "aplay"
        self.path = "../../GeneratedSoundFiles/"
        self.filename = "height4angle5.wav"

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
        delay = rospy.Duration(self.dist_to_delay(self.tracker, self.delay_coefficient))
        if rospy.Time.now() - self.last_tone < delay:
            return
        self.last_tone = rospy.Time.now()
        vol = self.angle_to_volume(self.tracker, self.volume_coefficient)
        atg = self.tracker.angle_to_go
        if atg >= 0:
            ratio=[0,1]
        else:
            ratio = [1,0]
        self.play_audio(vol, ratio)

    def dist_to_delay(self, tracker, coefficient, cutoff=4, reverse=False):
        """input the tracker and the coefficient. Takes the xy distance from the tracker, multiplies it by the coefficient, and returns the value - basically, the time to wait. For example, 2m -> 1m with coeff = 2 means 4s -> 2s delay."""
        xy_distance = tracker.xy_distance
        if xy_distance > cutoff:
            xy_distance = cutoff
        if not reverse:
            return xy_distance*coefficient
        else:
            return (cutoff * coefficient) - (xy_distance * coefficient)

    def angle_to_volume(self, tracker, coefficient, max_volume=40, reverse=False):
        atg = tracker.angle_to_go
        if not reverse:
            v = abs(atg*coefficient)
            if v > max_volume:
                v = max_volume
            return v
        else:
            v = max_volume - (coefficient*atg)
            if v < 0:
                v = 0
            elif v > max_volume:
                v = max_volume
            return v

    def play_audio(self, volume, ratio):
        cmd = 'amixer -D pulse sset Master {}%{}%'.format(volume*ratio[0], volume*ratio[1])
        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()
        cmd = "{} {}{}".format(self.player, self.path, self.filename)
        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()


class Offset_angle_and_distance():
    """
    Directs the user to a location offset from the object by [some distance] in the direction orthogonal to the target object. Uses (at present, at least) the same mappings as the other angle-disitance one.
    """

    def __init__(self, tracker):
        self.tracker = tracker
        self.isOn = False
        self.volume_coefficient = 1.0 #thing to change.
        self.delay_coefficient = 0.5 #thing to change.
        self.forward_offset = 0.5 #thing to change.
        self.right_offset = 0.0 #thing to change.
        self.last_tone = rospy.Time.now()
        self.player = "aplay"
        self.path = "../../GeneratedSoundFiles/"
        self.filename = "height4angle5.wav"
        self.offset_target_pub = rospy.Publisher("/offset_point", PointStamped, queue_size=10)

    def toggle(self):
        self.isOn = not self.isOn

    def turn_on(self):
        self.isOn = True

    def turn_off(self):
        self.isOn = False

    def call(self):
        if self.isOn:
            self.tracker.refresh_all()
            if self.tracker.xy_distance != None and self.tracker.target_surface_slope != None:
                self.run()

    def run(self):


        orthogonal_vector = (1.0, -(1.0/self.tracker.target_surface_slope))
        magnitude = math.sqrt(orthogonal_vector[0]**2 + orthogonal_vector[1]**2)
        normal_orthogonal_vector = (orthogonal_vector[0]/magnitude, orthogonal_vector[1]/magnitude)
        forward_offset_amount = (normal_orthogonal_vector[0]*self.forward_offset, normal_orthogonal_vector[1]*self.forward_offset)
        new_target_x = self.tracker.target_x + forward_offset_amount[0]
        new_target_y = self.tracker.target_y + forward_offset_amount[1]
        dx = self.tracker.x - new_target_x
        dy = self.tracker.y - new_target_y

        distance_to_target = math.sqrt(dy**2 + dy**2)

        atg = math.degrees(math.atan2(dy, dx) - self.tracker.yaw)
        max_angle = math.degrees(np.pi/2)
        if atg > max_angle:
            atg = max_angle
        elif atg < -max_angle:
            atg = -max_angle

        vol = min(abs(atg)*self.volume_coefficient, 40)

        point_msg = PointStamped(header=Header(frame_id="depth_camera"), point=Point(y=new_target_y, z=self.tracker.target_z, x=new_target_x))
        self.offset_target_pub.publish(point_msg)

        delay = rospy.Duration(min(distance_to_target*self.delay_coefficient, 4*self.delay_coefficient))
        if rospy.Time.now() - self.last_tone < delay:
            return
        self.last_tone = rospy.Time.now()

        if atg >= 0:
            ratio=[0,1]
        else:
            ratio=[1,0] #setting the left/right balance.
        self.play_audio(vol, side)

    def play_audio(self, volume, ratio):
        cmd = 'amixer -D pulse sset Master {}%{}%'.format(volume*ratio[0], volume*ratio[1])
        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()
        cmd = "{} {}{}".format(self.player, self.path, self.filename)
        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()






        #slope is "rise over run", or "dy / dx " So, the slope points in the direction/vector of <1, slope value>  Since orthogonal is the negative reciprocal slope, that means that a vector orthogonal to the target surface is: <1, -(1/slopeval)>. Norm this, then multiply said norm vector by the offset_distance. Add/subtract/etcf that from the original x and y.

if __name__ == "__main__":
    tt = Tango_tracker()
    offset = Offset_angle_and_distance(tt)
    offset.turn_on()
    while True:
        offset.call()

#hough line transform