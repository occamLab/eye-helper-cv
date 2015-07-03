#!/usr/bin/env python

from tango_tracker import Tango_tracker
import rospy
from geometry_msgs.msg import PointStamped, Point, Pose
from sensor_msgs.msg import CameraInfo, PointCloud, CompressedImage
import cv2
from std_msgs.msg import Header

class Landmark(object):
	"""
	Draft for setting up landmarks.
	Publishes Tango's position to /clicked_point topic as a target in order to save current location.
	"""
	def __init__(self,tracker):
		self.tracker=tracker
		self.landmarks=[]
		self.pub = rospy.Publisher("/clicked_point",PointStamped,queue_size=10)

		#----------Publishing and subscribing to topics/nodes----------
		# rospy.init_node('landmark_position')
		# rospy.Subscriber('/landmark_position', PointStamped, self.add_landmark)
		# self.pub = rospy.Publisher('/set_landmark', PointStamped, queue_size=10)

	def handle_button(self):
		"""
		When spacebar is down, gets Tango's position
		"""

		cv2.namedWindow("set_landmark")
		k = cv2.waitKey(5)
		if ord(' ') == (k & 255):
			self.add_landmark()
		for i in range(0,10):
			i=str(i)
			if ord(i) == (k & 255):
				self.publish_landmark(int(i)-1)


	def landmark_number(self):
		"""
		Returns the number of landmarks set
		"""
		return len(self.landmarks), self.landmarks

	def add_landmark(self):
		"""
		Publishes Tango's position to the /clicked_point topic
		"""
		self.landmarks.append((self.tracker.x, self.tracker.y, self.tracker.z))
		print self.landmarks
		
	def publish_landmark(self, i):
		point_msg = PointStamped(header=Header(frame_id="depth_camera"),point=Point(y=self.landmarks[i][1],z=self.landmarks[i][2],x=self.landmarks[i][0]))
		self.pub.publish(point_msg)


if __name__ == "__main__":
    tracker = Tango_tracker()
    lm=Landmark(tracker)
    r = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        tracker.refresh_all()
        lm.handle_button()