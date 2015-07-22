#!/usr/bin/env python

"""
Contains a "version 1" eye-helper, which connects distance to frequency and angle to volume/side.

"""
import rospy
import rospkg
import subprocess
from tango_tracker import Tango_tracker
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
import math
import numpy as np
import time
from std_msgs.msg import Header
from tf import TransformListener
from eye_helper.msg import Sound
import ransac

class Angle_and_distance():
    """
    maps volume to amount that the angle is off, side to angle, and distance to frequency.
    """
    def __init__(self, tracker):
        self.tracker = tracker
        self.isOn = False
        self.reverse = False #thing to change - changes whether sound-in-right-ear means move *to*, or *away from* the right.
        self.volume_coefficient = 2 #thing to change.
        self.minimum_volume = 15 #thing to change.
        self.max_volume = 40 #thing to change.
        self.delay_coefficient = 0.5 #thing to change
        self.last_tone = rospy.Time.now()
        self.player = "aplay"
        self.rospack = rospkg.RosPack();
        self.path = self.rospack.get_path('eye_helper') + '/../GeneratedSoundFiles/'
        self.filename = None 
        self.sound_pub=rospy.Publisher('/sound_info', Sound, queue_size=10)

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

        self.filename= "height4angle5.wav"
        vol = self.angle_to_volume(self.tracker, self.volume_coefficient)
        atg = self.tracker.angle_to_go
        if self.reverse:
            if atg >= 0:
                ratio = [0,1]
            else:
                ratio = [1,0]
        else:
            if atg >= 0:
                ratio=[1,0]
            else:
                ratio = [0,1]
        if abs(atg) * self.volume_coefficient < self.minimum_volume:
            vol = self.minimum_volume
            ratio = [1,1]
        elif abs(atg) * self.volume_coefficient > self.max_volume:
            vol = self.max_volume

        print self.tracker.xy_distance
        if abs(self.tracker.xy_distance)<=0.35: 
            vol=30
            ratio=[1,1]
            self.filename='You_arrived.wav'

        
        self.play_audio(vol, ratio)

        self.sound_info= Sound(file_path=self.path + self.filename,volume=float(vol),mix_left=float(ratio[0]),mix_right=float(ratio[1]) )
        self.sound_pub.publish(self.sound_info)


    def dist_to_delay(self, tracker, coefficient, cutoff=4, reverse=False):
        """input the tracker and the coefficient. Takes the xy distance from the tracker, multiplies it by the coefficient, and returns the value - basically, the time to wait. For example, 2m -> 1m with coeff = 2 means 4s -> 2s delay."""
        xy_distance = tracker.xy_distance
        if xy_distance > cutoff:
            xy_distance = cutoff
        if not reverse:
            return xy_distance*coefficient
        else:
            return (cutoff * coefficient) - (xy_distance * coefficient)

    def angle_to_volume(self, tracker, coefficient):
        atg = tracker.angle_to_go
        v = abs(atg*coefficient)
        return v


    def play_audio(self, volume, ratio):
        cmd = 'amixer -D pulse sset Master {}%,{}%'.format(volume*ratio[0], volume*ratio[1])
        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()
        cmd = "{} {}{}".format(self.player, self.path, self.filename)
        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()
        if self.filename == 'You_arrived.wav':
            rospy.sleep(10.)


class Offset_angle_and_distance():
    """
    Directs the user to a location offset from the object by [some distance] in the direction orthogonal to the target object. Uses (at present, at least) the same mappings as the other angle-disitance one.
    """

    def __init__(self, tracker):
        self.tracker = tracker
        self.isOn = False
        self.reverse = True
        self.volume_coefficient = 1.0 #thing to change.
        self.max_volume = 40
        self.minimum_volume = 15
        self.delay_coefficient = 0.5 #thing to change.
        self.forward_offset = -0.6 #thing to change.
        self.right_offset = -0.3 #thing to change.
        self.last_tone = rospy.Time.now()
        self.player = "aplay"
        self.rospack = rospkg.RosPack()
        self.path = self.rospack.get_path('eye_helper') + '/../GeneratedSoundFiles/'
        self.filename = "height4angle5.wav"
        self.offset_target_pub = rospy.Publisher("/offset_point", PointStamped, queue_size=10)
        self.sound_pub=rospy.Publisher('/sound_info', Sound, queue_size=10)
        # self.tf = TransformListener()
        self.line_id_pub = rospy.Publisher("/target_line", Marker, queue_size=10)

    def toggle(self):
        self.isOn = not self.isOn

    def turn_on(self):
        self.isOn = True

    def turn_off(self):
        self.isOn = False

    def call(self):
        if self.isOn:
            self.tracker.refresh_all()
            if self.tracker.xy_distance != None and self.tracker.target_surface_points != None and self.tracker.target_surface_slope != None:
                self.run()
            # else:
            #     print self.tracker.xy_distance, self.tracker.target_surface_slope, self.tracker.x

    def run(self):
        # good_points = ransac.ransac_2d(self.tracker.target_surface_points)
        good_points = self.tracker.ransac_points
        if good_points == None:
            parallel_v = (1.0, self.tracker.target_surface_slope)
            ortho_v = (1.0, -(1.0/self.tracker.target_surface_slope)) 
        else:
            parallel_v = (good_points[1][0]-good_points[0][0], good_points[1][1]-good_points[0][1])
            ortho_v = (1.0, (good_points[0][0]-good_points[1][0])/(good_points[1][1] - good_points[0][1]))

            line = Marker()
            line.type = Marker.LINE_STRIP
            line.points = [Point(z=self.tracker.target_z, x = good_points[0][0], y=good_points[0][1]), Point(z=self.tracker.target_z, x = good_points[1][0], y=good_points[1][1])]

        parallel_mag = math.sqrt(parallel_v[0]**2 + parallel_v[1]**2)
        ortho_mag = math.sqrt(ortho_v[0]**2 + ortho_v[1]**2)
        unit_parallel = (parallel_v[0]/parallel_mag, parallel_v[1]/parallel_mag)
        unit_ortho = (ortho_v[0]/ortho_mag, ortho_v[1]/ortho_mag)
        forward_offset_amount = (unit_ortho[0]*self.forward_offset, unit_ortho[1]*self.forward_offset)
        right_offset_amount = (unit_parallel[0]*self.right_offset, unit_parallel[1]*self.right_offset)

        if math.sqrt((self.tracker.target_x + forward_offset_amount[0]-self.tracker.x)**2 + (self.tracker.target_y + forward_offset_amount[1] - self.tracker.y)**2) < math.sqrt((self.tracker.target_x - forward_offset_amount[0] - self.tracker.x)**2 + (self.tracker.target_y - forward_offset_amount[1] - self.tracker.y)**2):
            new_target_x = self.tracker.target_x + forward_offset_amount[0]
            new_target_y = self.tracker.target_y + forward_offset_amount[1]
        #comparing which way reduces the distance to figure out forward/backwards
        else:
            new_target_x = self.tracker.target_x - forward_offset_amount[0]
            new_target_y = self.tracker.target_y - forward_offset_amount[1]
            #right now not using right/left until the forward/back thing is resolved.

        # new_target_x = self.tracker.target_x + forward_offset_amount[0] + right_offset_amount[0]
        # new_target_y = self.tracker.target_y + forward_offset_amount[1] + right_offset_amount[1]
        xtg = new_target_x - self.tracker.x
        ytg = new_target_y - self.tracker.y # tg = to-go.

        distance_to_target = math.sqrt(xtg**2 + ytg**2)

        atg = math.degrees(math.atan2(ytg, xtg) - self.tracker.yaw)
        max_angle = math.degrees(np.pi/2)
        if atg > max_angle:
            atg = max_angle
        elif atg < -max_angle:
            atg = -max_angle

        vol = min(abs(atg)*self.volume_coefficient, 40)
       
        point_msg = PointStamped(header=Header(stamp=self.tracker.pose_timestamp, frame_id="odom"), point=Point(y=new_target_y + self.tracker.y, z=self.tracker.target_z, x=new_target_x + self.tracker.x))
        self.offset_target_pub.publish(point_msg)

        delay = rospy.Duration(min(distance_to_target*self.delay_coefficient, 4*self.delay_coefficient))
        if rospy.Time.now() - self.last_tone < delay:
            return

        self.last_tone = rospy.Time.now()

        if self.reverse:
            if atg >= 0:
                ratio = [0,1]
            else:
                ratio = [1,0]
        else:
            if atg >= 0:
                ratio=[1,0]
            else:
                ratio = [0,1]
        if abs(atg) * self.volume_coefficient < self.minimum_volume:
            vol = self.minimum_volume
            ratio = [1,1]

        # self.play_audio(vol, ratio)

        self.sound_info= Sound(file_path=self.filename,volume=float(vol),mix_left=float(ratio[0]),mix_right=float(ratio[1]) )
        self.sound_pub.publish(self.sound_info)



    def play_audio(self, volume, ratio):
        cmd = 'amixer -D pulse sset Master {}%,{}%'.format(volume*ratio[0], volume*ratio[1])
        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()
        cmd = "{} {}{}".format(self.player, self.path, self.filename)
        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()



if __name__ == "__main__":
    tt = Tango_tracker()
    eh = Angle_and_distance(tt)
    eh.turn_on()
    while not rospy.is_shutdown():
        eh.call()

#todo: look into hough line transform
