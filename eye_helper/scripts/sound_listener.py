#!/usr/bin/env python

import rospy
from eye_helper.msg import Sound
import subprocess


def callback(data):
    player = 'aplay'
    cmd = 'amixer set Master {}'.format(data.volume)
    popen = subprocess.Popen(cmd, shell=True)
    popen.communicate()
    # need to scale this by volume as a percentage.... TODO
    cmd = 'amixer sset Headphone {}%,{}%'.format(data.mix_left,data.mix_right)
    popen = subprocess.Popen(cmd, shell=True)
    popen.communicate()
    # print self.filename
    cmd = '{} {}'.format(player, data.file_path)
    popen = subprocess.Popen(cmd, shell=True)
    popen.communicate()
    print data

def sound_listener():
	rospy.init_node('sound_listener', anonymous=True)
	rospy.Subscriber('/sound_info', Sound, callback)
	rospy.spin()


if __name__ == '__main__':
	sound_listener()