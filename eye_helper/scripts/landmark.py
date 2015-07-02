#!/usr/bin/env python

from tango_tracker import Tango_tracker
import rospy
import numpy as np
from geometry_msgs.msg import PointStamped, Point, Pose
from sensor_msgs.msg import CameraInfo, PointCloud, CompressedImage
import cv2
from std_msgs.msg import Header
from tf import TransformListener
from threading import Lock

class Landmark(object):
	"""
	Draft for setting up landmarks
	"""
	def __init__(self,tracker):
		self.tracker=tracker
		self.landmarks=[]
		self.pub = rospy.Publisher("/clicked_point",PointStamped,queue_size=10)
        # self.rospack = rospkg.RosPack() #May not need this?


		#----------Publishing and subscribing to topics/nodes----------

		# rospy.init_node('landmark_position')
		# rospy.Subscriber('/landmark_position', PointStamped, self.add_landmark)
		# self.pub = rospy.Publisher('/set_landmark', PointStamped, queue_size=10)

	def handle_button(self):
		"""
		When spacebar is down, gets Tango's position
		"""

		cv2.namedWindow("mywin")
		k = cv2.waitKey(5)
		if ord(' ') == (k & 255):
			self.add_landmark()
			print self.landmarks
			print self.landmark_number

	def landmark_number(self):
		"""
		Returns the number of landmarks set
		"""
		return len(self.landmarks)

	def add_landmark(self):
		"""
		blah blah
		"""
		self.landmarks.append((self.tracker.x, self.tracker.y, self.tracker.z))

		point_msg = PointStamped(header=Header(frame_id="depth_camera"),point=Point(y=self.tracker.y,z=self.tracker.z,x=self.tracker.x))
		self.pub.publish(point_msg)

		# point_msg = PointStamped(header=Header(stamp=msg.header.stamp,frame_id="depth_camera"),point=Point(y=tracker.y, z=tracker.z, x=tracker.x))
		# self.landmarks.append(point_msg)


if __name__ == "__main__":
    tracker = Tango_tracker()
    lm=Landmark(tracker)
    r = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        tracker.refresh_all()
        lm.handle_button()