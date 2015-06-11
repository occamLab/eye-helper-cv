#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
	rospy.loginfor(rospy.get_caller_id() + 'I heard %s', data.data)

def sound_listener():
	rospy.init_node('sound_listener', anonymous=True)
	rospy.Subscriber('/sound_info', String, callback)
	rospy.spin()


if __name__ == '__main__':
	sound_listener()