#!/usr/bin/env python

import numpy as np
import rospy
import math
import subprocess
import rospkg
#from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PointStamped
#import threading 
import time
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64, Float64MultiArray, String


class Tango_tracker():
    """
    Calculate relative locations w/ Tango. Refreshes target info with self.refresh_all().
    """
    def __init__(self):

#-----------------PARAMETERS-----------------------
        self.starting_x = None
        self.starting_y = None
        self.starting_z = None

        self.x = None
        self.y = None
        self.z = None

        self.target_x = None
        self.target_y = None
        self.target_z = None

        self.xy_distance = None
        self.z_distance = None
        self.angle_to_go = None

#----------------------ROS------------------------
        rospy.init_node('tangoposecalc')
        rospy.Subscriber('/tango_pose', PoseStamped, self.process_pose)
        rospy.Subscriber('/tango_angles', Float64MultiArray, self.process_angle)
        rospy.Subscriber('/clicked_point', PointStamped, self.set_target)
        self.logger = rospy.Publisher('/log', String, queue_size=10)
        self.rospack = rospkg.RosPack();
        self.path = self.rospack.get_path('eye_helper') + '/../GeneratedSoundFiles/'

#-------------------PLAYBACK----------------------
        self.player = 'aplay'
        self.playback_interval = rospy.Duration(0.2)
        self.last_tone = rospy.Time.now()
        self.angle_to_volume_dictionary = {0: 21, 5: 22, 10: 23, 15: 24, 20: 25, 25: 26, 30: 27, 35: 28, 40: 29, 45: 30, 50: 31, 55:31, 60:31, 65:31, 70:31, 75 : 31, 80 : 31, 85 : 31, 90: 31}

#---------------PROCESS-INPUTS--------------------
    def process_pose(self, msg):
        """
        zeroes position data, then writes it to class varirables.
        """
        if self.starting_x == None:
            self.starting_x = msg.pose.position.x
            self.starting_y = msg.pose.position.y
            self.starting_z = msg.pose.position.z
        self.x = msg.pose.position.x - self.starting_x
        self.y = msg.pose.position.y - self.starting_y
        self.z = msg.pose.position.z - self.starting_z

    def process_angle(self, msg):
        """
        writes angle info to class variables.
        """
        self.yaw = msg.data[2]
        self.pitch = msg.data[1]
        self.roll = msg.data[0]

    def set_target(self, msg):
        """
        writes the message info to the target.
        """
        self.target_x = msg.point.x - self.starting_x
        self.target_y = msg.point.y - self.starting_y
        self.target_z = msg.point.z - self.starting_z

#--------------GENERATE-OUTPUTS---------------------
    def refresh_xy_distance(self):
        self.xy_distance = math.sqrt((self.target_x - self.x)**2 + (self.target_y - self.y)**2)

    def refresh_z_distance(self):
        self.z_distance = self.target_z - self.z

    def refresh_angle():
        max_angle = math.degrees(np.pi / 2)
        min_angle = -max_angle
        atg = math.degrees(math.atan2(self.target_y-self.y, self.target_x-self.x) - self.yaw)
        if atg < min_angle:
            atg = min_angle
        elif atg > max_angle:
            atg = max_angle
        self.angle_to_go = atg

    def refresh_all(self):
        self.refresh_xy_distance()
        self.refresh_z_distance()
        self.refresh_angle()



if __name__ == "__main__":
    eh = TangoPoseCalc()
    r = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        r.sleep()
        eh.refresh_all()
        print "xy distance: " + str(eh.xy_distance) + "\tz distance: " + str(eh.z_distance) + "\tangle: " + str(eh.angle_to_go)