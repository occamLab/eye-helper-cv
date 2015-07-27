#!/usr/bin/env python

import rospy
import rospkg
from geometry_msgs.msg import PointStamped, Point, Pose
from std_msgs.msg import Int32, Int32MultiArray, Bool, Header
from sensor_msgs.msg import Joy, JoyFeedback, CameraInfo, PointCloud, CompressedImage
from tango_tracker import Tango_tracker
import xwiimote
import errno
from select import poll, POLLIN
import math
import time

class Landmark():
	"""
	Putting together Breadcrumb_tracker, landmark and wiimote. 
	A class that enables people to chose landmark positions and some pinpoints for the trail.
	Switches between trails to guide the person to the landmark through a certain path.
	Controlled by using wiimote. 
	Just a draft for now.
	"""

	def __init__(self,tracker):
		self.tracker=tracker
		self.states=['START_UP', 'INITIALIZING_LANDMARK', 'ADDING_TRAILS', 'TOGGLING_LANDMARKS', 'PUBLISHING_TRAILS']
		self.state='START_UP'
		self.rospack = rospkg.RosPack()
		self.filepath= self.rospack.get_path('eye_helper') + "/../GeneratedSoundFiles/Landmark_sound_files/"

		#Landmarks and trails:
		self.landmark_options=[]
		self.landmarks={}
		self.trail=[]
		self.landmark_name=None
		self.landmark_number=1
		self.published_landmark = None

		#Publishing stuff
		self.pub = rospy.Publisher("/landmark_point",PointStamped,queue_size=10)
		rospy.Subscriber('/wii_buttons',Int32 , self.handle_button)
		self.data=None
		self.threshold = 0.15 #thing to change
		#self.last_added=rospy.Publisher("/last_landmark", PointStamped, queue_size=10)

	def handle_button(self,msg):
		self.data = msg.data
		if msg.data ==4: 
			self.state='INITIALIZING_LANDMARK'
		if msg.data == 9:
			self.state='ADDING_TRAILS'
		if msg.data == 2 or msg.data ==3:
			self.state='TOGGLING_LANDMARKS'
		if msg.data == 8: #fix this
			self.state='PUBLISHING_TRAILS'

	def run(self):
		if self.state == 'START_UP':
			pass

		if self.state == 'INITIALIZING_LANDMARK':
			self.initiate_landmark()
			self.state = 'START_UP'

		if self.state == 'ADDING_TRAILS':
			self.add_trail()
			self.state = 'START_UP'

		if self.state == 'TOGGLING_LANDMARKS':
			self.landmark_menu()
			self.state = 'START_UP'

		if self.state == 'PUBLISHING_TRAILS':
			self.publish_trail()
			self.state = 'START_UP'

		if self.state not in self.states:
			print 'State Error: self.state value is invalid'

	def initiate_landmark(self):
		self.landmark_name='landmark'+str(len(self.landmarks)+1)
		self.landmarks[self.landmark_name]=[]
		print 'Initialized landmark' + str(self.landmark_name) # need to make this into a sound file.
		self.data=None

	def add_trail(self):
		current_position=(self.tracker.x, self.tracker.y, self.tracker.z)
		trail_list = self.landmarks[self.landmark_name]
	 	trail_list.append(current_position)
	 	print self.landmarks
	 	print 'added point on trail for landmark'+ str(self.landmark_name) # need to make this into a sound file.
	 	self.data=None

	def landmark_menu(self):
		if self.data == 2: #up
			self.landmark_number=self.landmark_number-1
			self.data=None
		if self.data == 3: #down
			self.landmark_number += 1
			self.data=None
		print 'selected landmark:', 'landmark', self.landmark_number
		return self.landmark_number

	def publish_trail(self):
		trails = self.landmarks['landmark'+str(self.landmark_number)]
		print trails
		trail_number=0
		if len(trails) >0:
			while trail_number>=0 or trail_number <len(trails):
				point_msg = PointStamped(header=Header(frame_id="odom", stamp=self.tracker.pose_timestamp), point=Point(x=trails[trail_number][0],y=trails[trail_number][1],z=trails[trail_number][2]))
				self.pub.publish(point_msg)
				print (trails[trail_number][0],trails[trail_number][1],trails[trail_number][2])
				if self.tracker.xy_distance<=0.15 and trail_number != len(trails)-1:
					trail_number += 1
				# if trail_number == len(trails)-1:
				# 	print "You've arrived" #Turn it into a sound file
		else:
			print 'Error: No trail points!'
		self.data=None

	# def drop_(self):
	# 	current_position=(self.tracker.x, self.tracker.y, self.tracker.z)
	# 	self.landmarks['landmark'+str(len(self.landmarks)+1)]=[]
	# 	msg=PointStamped(header=Header(frame_id="odom", stamp=self.tracker.pose_timestamp), point=Point(y=self.tracker.y,z=self.tracker.z,x=self.tracker.x))
	# 	self.pub.publish(msg)
	# 	print self.landmarks
	# 	print self.landmark_number

	# def add_trail(self):
	# 	current_position=(self.tracker.x, self.tracker.y, self.tracker.z)
	# 	self.tracker.append(current_position)

	# def landmark_number(self):
	# 	"""
	# 	Returns the total number of landmarks and landmark positions
	# 	"""
	# 	return len(self.landmarks)

	# def add_trail(self):
	# # 	self.tracker.trail.append((self.tracker.x, self.tracker.y, self.tracker.z))
	# def handle_wii_keys(self):
	# 	(code, ) = self.wiimote.event.get_key()
	# 	if code == 1:
	# 		self.tracker.drop_breadcrumb()
	# 	elif code == 2:
	# 		self.tracker.pick_up_breadcrumb()

	# 	elif 

	# 	else: pass

		# cv2.namedWindow("set_landmark")
		# k = cv2.waitKey(5)
		# if ord(' ') == (k & 255):
		# 	self.add_landmark()
		# for i in range(0,10):
		# 	if ord(str(i)) == (k & 255):
		# 		print 'landmark published'
		# 		self.publish_landmark(i)

		# Will get code for 
		# cv2.namedWindow("set_landmark")
		# k = cv2.waitKey(5)
		# if ord(' ') == (k & 255):
		# 	self.add_landmark()
		# for i in range(0,10):
		# 	if ord(str(i)) == (k & 255):
		# 		print 'landmark published'
		# 		self.publish_landmark(i)

	# def add_landmark(self):
	# 	"""
	# 	Stores Tango's position in landmarks list
	# 	"""
	# 	self.landmarks.append((self.tracker.pose_x, self.tracker.pose_y, self.tracker.pose_z))
	# 	msg=PointStamped(header=Header(frame_id="odom", stamp=self.tracker.pose_timestamp), point=Point(y=self.tracker.pose_y,z=self.tracker.pose_z,x=self.tracker.pose_x))
	# 	self.last_added.publish(msg)
	# 	print self.landmarks
	# 	print self.landmark_number

	# def publish_landmark(self, i):
	# 	"""
	# 	Publishes selected landmark position to /clicked_point topic
	# 	"""
	# 	point_msg = PointStamped(header=Header(frame_id="odom", stamp=self.tracker.pose_timestamp), point=Point(y=self.landmarks[i-1][1],z=self.landmarks[i-1][2],x=self.landmarks[i-1][0]))
	# 	self.pub.publish(point_msg)


if __name__ == "__main__":
    tt = Tango_tracker('tango_tracker2')
    lm=Landmark(tracker=tt)
    r = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        tt.refresh_all()
        lm.run()
        r.sleep()