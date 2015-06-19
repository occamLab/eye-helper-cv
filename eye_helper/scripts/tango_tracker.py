#!/usr/bin/env python

import numpy as np
from scipy.stats import linregress
import rospy
import math
import subprocess
import rospkg
from geometry_msgs.msg import PoseStamped, PointStamped, Point32
from sensor_msgs.msg import PointCloud
import time
from tf import TransformListener
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64, Float64MultiArray, String


class Tango_tracker():
    """
    Calculate relative locations w/ Tango. Refreshes target info with self.refresh_all().
    """
    def __init__(self):

#-----------------PARAMETERS-----------------------
# This section just initializes the parameters to None.
        self.starting_x = None
        self.starting_y = None
        self.starting_z = None

        self.x = None
        self.y = None
        self.z = None

        self.yaw = None
        self.pitch = None
        self.roll = None

        self.target_x = None
        self.target_y = None
        self.target_z = None

        self.target_surface_slope = None
        self.pose_timestamp = None


#---------above is input; below is "output"---------
        self.xy_distance = None
        self.z_distance = None
        self.angle_to_go = None

        self.forward_distance = None
        self.right_distance = None

#----------------------ROS------------------------
# This sets the tango tracker to subscribe to the relevant topics, and process them properly.
        rospy.init_node('tango_tracker')
        rospy.Subscriber('/tango_pose', PoseStamped, self.process_pose)
        rospy.Subscriber('/tango_angles', Float64MultiArray, self.process_angle)
        rospy.Subscriber('/clicked_point', PointStamped, self.set_target)
        rospy.Subscriber('/nearby_cloud', PointCloud, self.process_points_near_target)
        self.logger = rospy.Publisher('/log', String, queue_size=10)
        self.rospack = rospkg.RosPack();
        self.tf = TransformListener()
        self.path = self.rospack.get_path('eye_helper') + '/../GeneratedSoundFiles/'

#---------------PROCESS-INPUTS--------------------
# Converts tango messages into things like x position, yaw, etc.
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
        self.pose_timestamp = msg.header.stamp

    def process_angle(self, msg):
        """
        writes angle info to class variables.
        """
        self.yaw = msg.data[2]
        self.pitch = msg.data[1]
        self.roll = msg.data[0]

    def process_points_near_target(self, msg):
        points = msg.points
        # self.tf.waitForTransform("depth_camera", "odom", self.pose_timestamp, rospy.Duration(1.0))
        # transformed_points = [self.tf.transformPoint('odom', i) for i in points]
        xvals = [i.x for i in points]
        yvals = [i.y for i in points]
        zvals = [i.z for z in points]
        slope, intercept, r, p, err = linregress(xvals, yvals)
        self.target_surface_slope = slope

    def set_target(self, msg):
        """
        writes the message info to the target.
        """
        self.target_x = msg.point.x - self.starting_x
        self.target_y = msg.point.y - self.starting_y
        self.target_z = msg.point.z - self.starting_z
        print "target set"

#--------------GENERATE-OUTPUTS---------------------
# Updates output variables, like self.xy_distance.
    def refresh_xy_distance(self):
        self.xy_distance = math.sqrt((self.target_x - self.x)**2 + (self.target_y - self.y)**2)

    def refresh_z_distance(self):
        self.z_distance = self.target_z - self.z

    def refresh_angle(self):
        max_angle = 90 #degrees
        min_angle = -90 #degrees
        # xy_angle_to_target = math.degrees(math.atan2(self.target_y - self.y, self.target_x - self.x)) #-180 to 180
        # yaw_deg = math.degrees(self.yaw) #also -180 to 180
        atg = math.degrees(math.atan2(self.target_y-self.y, self.target_x-self.x) - self.yaw)
        if 90 < atg <= 180:
            atg = 90
        elif 180 < atg <= 270:
            atg = -90
        elif 270 < atg <= 360:
            atg = 360 - atg
        elif -360 < atg <= -270:
            atg = atg + 360
        elif -270 < atg <= -180:
            atg = 90
        elif -180 < atg <= -90:
            atg = -90

        self.angle_to_go = atg

    def refresh_orthogonal_distances(self):
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        self.forward_distance = dx*math.cos(self.yaw) + dy*math.sin(self.yaw)
        self.right_distance = dx*math.cos(self.yaw-(np.pi/2)) + dy*math.sin(self.yaw-(np.pi/2))


    def refresh_all(self):
        if self.z == None or self.target_z == None or self.yaw == None:
            return
        self.refresh_xy_distance()
        self.refresh_z_distance()
        self.refresh_angle()
        self.refresh_orthogonal_distances()



if __name__ == "__main__":
    eh = Tango_tracker()
    r = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        r.sleep()
        eh.refresh_all()
        print "xy distance: {:.3} \t z distance: {:.3} \t angle: {:.3} \t fwd: {:.3} \t right: {:.3}".format(eh.xy_distance, eh.z_distance, eh.angle_to_go, eh.forward_distance, eh.right_distance)