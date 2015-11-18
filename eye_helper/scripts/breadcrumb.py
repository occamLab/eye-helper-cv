#!/usr/bin/env Python

"""
landmark.py, w/ breadcrumb functionality thing. Just a draft right now.
"""

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
from std_msgs.msg import Float64, Float64MultiArray, String, Int32


class Breadcrumb_tracker():
    """
    A tracker, but one that switches targets on-the-fly. Instead of using clicked points, uses a list of targets - its "trail."
    """
    def __init__(self, nodename='breadcrumb_tracker'):

    #-----------------PARAMETERS-----------------------
    # This section just initializes the parameters to None.
        self.x = None
        self.y = None
        self.z = None

        self.yaw = None
        self.pitch = None
        self.roll = None

        self.target_x = None
        self.target_y = None
        self.target_z = None

        self.target_surface_points = None
        self.target_surface_slope = None
        self.pose_timestamp = None

        self.trail = []
        self.onTrail = False
        self.threshold = 0.1 # the distance which is "close enough" to a point to switch from.

    #---------above is input; below is "output"---------
        self.xy_distance = None
        self.z_distance = None
        self.angle_to_go = None

        self.forward_distance = None
        self.right_distance = None

    #----------------------ROS------------------------
    # This sets the tango tracker to subscribe to the relevant topics, and process them properly.
        rospy.init_node(nodename)
        rospy.Subscriber('/tango_pose', PoseStamped, self.process_pose)
        rospy.Subscriber('/tango_angles', Float64MultiArray, self.process_angle)
        rospy.Subscriber('/wii_buttons', Int32, self.process_button)
        rospy.Subscriber('/android_buttons', String, self.process_button)
        self.logger = rospy.Publisher('/log', String, queue_size=10)
        self.breadcrumb_trail_pub = rospy.Publisher('/breadcrumbs', PointCloud, queue_size=10)
        self.rospack = rospkg.RosPack();
        self.tf = TransformListener()
        self.path = self.rospack.get_path('eye_helper') + '/../GeneratedSoundFiles/'

    def process_pose(self, msg):
        """
        zeroes position data, then writes it to class variables.
        """
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        self.pose_timestamp = msg.header.stamp

    def process_angle(self, msg):
        """
        writes angle info to class variables.
        """
        self.yaw = msg.data[2]
        self.pitch = msg.data[1]
        self.roll = msg.data[0]

    def set_target(self, point):
        """
        writes the message info to the target.
        """
        self.target_x = point[0]
        self.target_y = point[1]
        self.target_z = point[2]


    def drop_breadcrumb(self):
        current_point = (self.x, self.y, self.z)
        self.trail.append(current_point)
        print "breadcrumb dropped"
        self.onTrail = False

    def pick_up_breadcrumb(self):
        if len(self.trail) == 0:
            print "trail over"
            self.onTrail = False
            return
        else:
            self.set_target(self.trail[-1])
            del self.trail[-1]
            print "new target"
            self.onTrail = True
        self.publish_trail()

    def process_button(self, msg):
        if msg.data == 6:
            self.drop_breadcrumb()
            self.publish_trail()
        elif msg.data == 7:
            self.pick_up_breadcrumb()
            
        elif msg.data == "Start":
            self.trail = []
            self.drop_breadcrumb()
            self.publish_trail()
        elif msg.data == "Drop":
            self.drop_breadcrumb()
            self.publish_trail()
        elif msg.data == "Pickup":
            self.pick_up_breadcrumb()


    def set_landmark_target(self,msg):
        current_point=(self.x, self.y, self.z)
        self.landmarks[current_point]=[] # what's it mapping to...?

    def publish_trail(self):
        pc = PointCloud()
        pc.header.frame_id = 'odom'
        for p in self.trail:
            pc.points.append(Point32(x=p[0], y=p[1], z=p[2]))
        self.breadcrumb_trail_pub.publish(pc)
        print "trail published"


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
        if self.z == None or self.target_z == None or self.yaw == None or not self.onTrail:
            return
        self.refresh_xy_distance()
        if self.xy_distance < self.threshold:
            self.pick_up_breadcrumb()
            self.refresh_xy_distance()

        self.refresh_z_distance()
        self.refresh_angle()
        self.refresh_orthogonal_distances()




if __name__ == "__main__":
    eh = Breadcrumb_tracker()
    r = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        r.sleep()
        eh.refresh_all()
        print "xy distance: {:.3} \t z distance: {:.3} \t angle: {:.3} \t fwd: {:.3} \t right: {:.3}".format(eh.xy_distance, eh.z_distance, eh.angle_to_go, eh.forward_distance, eh.right_distance)