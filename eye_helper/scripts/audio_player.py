#!/usr/bin/env python

import cv2
import numpy as np
import subprocess

class AudioPlayer():

    def __init__(self):
        self.angle = 85
        self.height = 4
        self.path = "../GeneratedSoundFiles/"
        self.filename = "{}height{}angle{}.wav".format(self.path, self.height, self.angle)
        self.player = 'aplay'

    def audio_loop(self, center):
        while True:
            time.sleep(0.5)
            center = queue.get(block=False)
            if center != None:
                self.filename = play_audio(center, self.filename)
                print self.filename

    def play_wave(self):
        """
        plays an inputted wav file
        """
        
        print self.filename
        cmd = '{} {}'.format(self.player, self.filename)
        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()

    def myround(self,x, base=5):
        return int(base * round(float(x)/base))

    def play_audio(self, center):
        """
        plays a generated audio file based on the inputted center
        inputs: center - the center of the keypoints for a tracked image (x,y)
        """

        w = 512 # TODO: put in width of image
        max_height = 512 # TODO: height of the image
        
        max_angle = np.pi / 2
        min_angle = -max_angle
        self.angle = int(self.myround((center[0]/640.0) * 170.0 - 85.0))
        self.height = 4 # hard-coded at 4 for current testing
        
        if self.angle % 10 == 0:
            pass
        else:
            if self.angle > 0:
                self.filename = "{}height{}angle{}.wav".format(self.path, self.height, self.angle)
            else:
                self.filename = "{}height{}angle_{}.wav".format(self.path, self.height, abs(self.angle))
        print self.angle
        print self.height
        self.play_wave()

if __name__ == '__main__':
    ap = AudioPlayer()
    ap.play_audio((10, 10))