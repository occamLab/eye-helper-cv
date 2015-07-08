#!/usr/bin/env python

from tango_tracker import Tango_tracker
import rospy
from geometry_msgs.msg import PointStamped, Point, Pose
from sensor_msgs.msg import CameraInfo, PointCloud, CompressedImage
import cv2
from std_msgs.msg import Header

class Landmark():
	"""
	Draft for setting up landmarks.
	Publishes Tango's position to /landmark_point topic as a target in order to save current location.
	"""
	def __init__(self,tracker):
		self.tracker=tracker
		self.landmarks=[]
		self.pub = rospy.Publisher("/landmark_point",PointStamped,queue_size=10)
		self.last_added=rospy.Publisher("/last_landmark", PointStamped, queue_size=10)

		# rospy.init_node('landmark_position')
		# rospy.Subscriber('/landmark_position', PointStamped, self.add_landmark)
		# self.landpub = rospy.Publisher('/landmark_position', PointStamped, queue_size=10)

	def handle_button(self):
		"""
		When spacebar is down, stores Tango's position. When hit a number key, releases that one in the landmarks list
		"""

		cv2.namedWindow("set_landmark")
		k = cv2.waitKey(5)
		if ord(' ') == (k & 255):
			self.add_landmark()
		for i in range(0,10):
			if ord(str(i)) == (k & 255):
				print 'landmark published'
				self.publish_landmark(i)


	def landmark_number(self):
		"""
		Returns the total number of landmarks and landmark positions
		"""
		return len(self.landmarks)

	def add_landmark(self):
		"""
		Stores Tango's position in landmarks list
		"""
		self.landmarks.append((self.tracker.pose_x, self.tracker.pose_y, self.tracker.pose_z))
		msg=PointStamped(header=Header(frame_id="odom", stamp=self.tracker.pose_timestamp), point=Point(y=self.tracker.pose_y,z=self.tracker.pose_z,x=self.tracker.pose_x))
		self.last_added.publish(msg)
		print self.landmarks
		print self.landmark_number

	def publish_landmark(self, i):
		"""
		Publishes selected landmark position to /clicked_point topic
		"""
		point_msg = PointStamped(header=Header(frame_id="odom", stamp=self.tracker.pose_timestamp), point=Point(y=self.landmarks[i-1][1],z=self.landmarks[i-1][2],x=self.landmarks[i-1][0]))
		self.pub.publish(point_msg)


if __name__ == "__main__":
    tracker = Tango_tracker('tango_tracker2')
    lm=Landmark(tracker)
    r = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        tracker.refresh_all()
        lm.handle_button()