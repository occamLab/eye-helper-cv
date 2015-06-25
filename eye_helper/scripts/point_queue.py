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
import cv2

class Point_queue():
    """
    Calculate relative locations w/ Tango. Refreshes target info with self.refresh_all().
    """
    def __init__(self):
#----------------------ROS------------------------
# This sets the tango tracker to subscribe to the relevant topics, and process them properly.
        rospy.init_node('tango_tracker')
        rospy.Subscriber('/point_queue', PointStamped, self.add_target)
        self.pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=10)
        self.targets = []
        self.rospack = rospkg.RosPack();
        self.last_target = None
#---------------PROCESS-INPUTS--------------------

    def add_target(self, msg):
        """
        writes the message info to the target.
        """
        self.last_target = msg

    def incorporate_last_target(self):
        if self.last_target:
            self.targets.append(self.last_target)
  
    def publish_target(self):
        if len(self.targets):
            self.pub.publish(self.targets[0])
            self.targets = self.targets[1:]

    def queue_size(self):
        return len(self.targets)

if __name__ == "__main__":
    pq = Point_queue()
    r = rospy.Rate(5) # 5hz
    cv2.namedWindow("mywin")
    while not rospy.is_shutdown():
        k = cv2.waitKey(5)
        if ord(' ') == (k & 255):
            pq.publish_target()
        elif ord('a') == (k & 255):
            pq.incorporate_last_target()
        print pq.queue_size()
        r.sleep()