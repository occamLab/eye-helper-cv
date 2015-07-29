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
import subprocess

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
		self.filename = None
		self.path=None
		self.player= "aplay"
		self.isOnTrail = False

		#Landmarks and trails:
		self.landmark_options=[]
		self.landmarks={}
		self.trail=[]
		self.landmark_name=None
		self.landmark_number=1
		self.published_landmark = None

		#Publishing stuff
		self.pub = rospy.Publisher("/clicked_point",PointStamped,queue_size=10)
		rospy.Subscriber('/wii_buttons',Int32 , self.handle_button)
		self.data=None
		self.threshold = 0.15 #thing to change
		#self.last_added=rospy.Publisher("/last_landmark", PointStamped, queue_size=10)\

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
			if self.isOnTrail==True and self.tracker.xy_distance<0.3:
				self.publish_trail()
			else:
				pass

		if self.state == 'INITIALIZING_LANDMARK':
			self.initiate_landmark()
			self.audio_play()
			self.state = 'START_UP'

		if self.state == 'ADDING_TRAILS':
			self.add_trail()
			self.audio_play()
			self.state = 'START_UP'

		if self.state == 'TOGGLING_LANDMARKS':
			self.landmark_menu()
			self.state = 'START_UP'

		if self.state == 'PUBLISHING_TRAILS':
			self.publish_trail()
			self.state='START_UP'
			if len(self.landmarks['landmark'+str(self.landmark_number)])==0:
				self.isOnTrail=False
			
			# self.audio_play()

		if self.state not in self.states:
			print 'State Error: self.state value is invalid'

	def initiate_landmark(self):
		self.landmark_name='landmark'+str(len(self.landmarks)+1)
		self.landmarks[self.landmark_name]=[]
		print 'Initialized ' + str(self.landmark_name) # need to make this into a sound file.
		self.filename='initialized_landmark'
		self.data=None

	def add_trail(self):
		current_position=(self.tracker.x, self.tracker.y, self.tracker.z)
		trail_list = self.landmarks[self.landmark_name]
	 	trail_list.append(current_position)
	 	print self.landmarks
	 	print 'added point on trail for landmark'+ str(self.landmark_name) # need to make this into a sound file.
	 	self.filename='Added_trail'
	 	self.data=None

	def landmark_menu(self):
		if self.data == 2: #up
			self.landmark_number=self.landmark_number-1
			self.data=None
		if self.data == 3: #down
			self.landmark_number += 1
			self.data=None
		self.filename='landmark'+str(self.landmark_number)
		self.audio_play()
		print 'selected landmark:', 'landmark', self.landmark_number
		return self.landmark_number

	def publish_trail(self):
		self.isOnTrail=True
		trail= self.landmarks['landmark'+str(self.landmark_number)]
		if len(trail) == 0:
			print "trail_over"
		point_msg = PointStamped(header=Header(frame_id="odom", stamp=self.tracker.pose_timestamp), point=Point(x=trail[-1][0],y=trail[-1][1],z=trail[-1][2]))
		print (trail[-1][0],trail[-1][1],trail[-1][2])
		self.tracker.target_x=trail[-1][0]
		self.tracker.target_y=trail[-1][1]
		self.tracker.target_z=trail[-1][2]
		self.pub.publish(point_msg)
		self.tracker.refresh_all()
		trail.remove(trail[-1])
		print self.tracker.xy_distance
		# if self.tracker.xy_distance<0.2 and len(trail)>0:
		# 	self.publish_trail()

	def audio_play(self):
		p = subprocess.Popen('amixer -D pulse sset Master 30%', shell=True)
		p.communicate()
		self.path = self.filepath + self.filename
		p = subprocess.Popen('{} {}.wav'.format(self.player, self.path), shell=True)
		p.communicate()

if __name__ == "__main__":
    tt = Tango_tracker('tango_tracker2')
    lm=Landmark(tracker=tt)
    r = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        tt.refresh_all()
        lm.run()
        r.sleep()