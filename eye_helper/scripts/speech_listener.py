#!/usr/bin/env python

import rospy
from eye_helper.msg import Sound
import subprocess


def callback(data):
    player = 'aplay'
    cmd = '{} {}'.format(player, data.file_path)
    popen = subprocess.Popen(cmd, shell=True)
    popen.communicate()
    print data

def sound_listener():
	rospy.init_node('speech_listener', anonymous=True)
	rospy.Subscriber('/speech_info', Speech, callback)
	rospy.spin()


if __name__ == '__main__':
	sound_listener()