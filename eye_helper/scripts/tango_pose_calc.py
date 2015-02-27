#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import math
from cv_bridge import CvBridge, CvBridgeError
from image_selector import *
from object_matcher import *
from audio_player import *
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import threading 
import time
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from std_msgs.msg import Float64

class TangoPoseCalc():
    """
    Calculate relative locations w/ Tango
    """

    def __init__(self):
        rospy.init_node('tangoposecalc')
        rospy.Subscriber('/tango_pose', PoseStamped, self.process_pose)
        self.pub = rospy.Publisher('/yaw', Float64)
        # Stuff for Tango coord use:
        self.starting_x = None
        self.starting_y = None
        self.starting_z = None # Gets calibrated early; zeroes x, y, and z values.
        self.x = None
        self.y = None
        self.z = None # Gets overwritten ASAP.
        self.target_x = None
        self.target_y = None
        self.target_z = None # Volunteer would write it; for tests, could set arbitrarily and then set up testing space for it.
        self.need_to_go_x = None
        self.need_to_go_y = None
        self.need_to_go_z = None #Deltas; get calculated soon.
        self.quat_x = None
        self.quat_y = None
        self.quat_z = None
        self.quat_w = None # Tango's quaternion values, to get the user's current angle in its coordinate system. This angle plus the need_to_go values then gets turned into the angle from the user's current location & direction to the target coords.


    def quat_to_euler(self, quat_tuple):
        """
        different conversion method. Tuple of the form (x, y, z, w); returns tuple of the form (phi, theta, psi).
        """
        x = quat_tuple[0]
        y = quat_tuple[1]
        z = quat_tuple[2]
        w = quat_tuple[3]
        yaw = math.atan2(2.0 * (y*z + 2*x),w**2 - x**2 - y**2 + z**2)
        pitch = math.asin(-2.0 * (x*z - w*y))
        roll = math.atan2(2.0*(x*y + w*z),w**2 + x**2 - y**2 - z**2)
        return (yaw, pitch, roll)


    def process_pose(self, msg):
        # # print euler_from_quaternion([msg.pose.orientation.x,
        #                              msg.pose.orientation.y,
        #                              msg.pose.orientation.z,
        #                              msg.pose.orientation.w]);
        if self.starting_x == None:
            self.starting_x = msg.pose.position.x
            self.starting_y = msg.pose.position.y
            self.starting_z = msg.pose.position.z
        self.x = msg.pose.position.x - self.starting_x
        self.y = msg.pose.position.y - self.starting_y
        self.z = msg.pose.position.z - self.starting_z
        self.quat_x = msg.pose.orientation.x
        self.quat_y = msg.pose.orientation.y
        self.quat_z = msg.pose.orientation.z
        self.quat_w = msg.pose.orientation.w
        orientation_tuple = (self.quat_x,self.quat_y,self.quat_z,self.quat_w)
        # angles = self.quat_to_euler(orientation_tuple)
        angles = euler_from_quaternion(orientation_tuple)
        print("I currently think the x-y angle is: " + str(int(math.degrees(angles[1]))))
        #angles[2] is actually different!!! when it's flat down it's +/- 180; flat up, 0. That partly explains it.
        msg = Float64(data=angles[2])
        self.pub.publish(msg)
        # print "angles ", angles #still ain't quite working--not getting useful "real" angles from the quaternion values...

    def update_need_to_go(self):
        """
        Needs a target already set under self.target_x, target_y, and target_z
        Writes the delta X, Y, and Z needed to get there, to self.need_to_go_
        """
        self.need_to_go_x = self.target_x - self.x
        self.need_to_go_y = self.target_y - self.y
        self.need_to_go_z = self.target_z - self.z

    def tango_get_orientation(self):
        """
            Converts the ros feed's quaternion to euler using magical builtin stuff.
            Sets user's angle on XY (floor) plane as self.theta.
            Note to self: RADIANS
            Also, still not quite working. Angles are changing, but not in the way I'd expect them to.
        """
        orientation_tuple = (self.quat_x,self.quat_y,self.quat_z,self.quat_w)
        angles = euler_from_quaternion(orientation_tuple)
        self.theta = angles[2]
        print("I currently think the x-y angle is: " + str(math.degrees(self.theta)))

    def tango_get_angle_to_go(self):
        """
            takes self.theta (user's xy-angle) and self.need_to_go_y and x; uses atan2.
            Writes to self.need_to_go_angle (now in DEGREES).
        """
        self.need_to_go_angle = math.degrees(math.atan2(self.need_to_go_y, self.need_to_go_x) - self.theta)


if __name__ == "__main__":
    eh = TangoPoseCalc()
    r = rospy.Rate(5) # 5hz

    while not rospy.is_shutdown():
        r.sleep()