#!/usr/bin/env python

from tango_tracker import Tango_tracker
import cv2
import rospy

class Landmark(object):
	"""
	Draft for setting up landmarks
	"""
	def __init__(self,tracker):
		self.tracker=tracker
		self.landmarks=[]
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
		self.landmarks.append((self.tracker.x,self.tracker.y,self.tracker.z))


if __name__ == "__main__":
    tracker = Tango_tracker()
    eh=Landmark(tracker)
    r = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        tracker.refresh_all()
        eh.handle_button()