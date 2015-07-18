#!/usr/bin/env python

"""
trying out the xwiimote pointer thingie.
"""

import rospy
from sensor_msgs.msg import Joy, JoyFeedback #no idea what this actually is; doesn't seem to get used, but anyways.
import errno # ditto
from select import poll, POLLIN
import math
import xwiimote
import time
from std_msgs.msg import Int32, Int32MultiArray, Bool, Header #might not use header or the floats actually, we'll see.
from tango_tracker import Tango_tracker
import numpy as np


class Wii_pointer():
    def __init__(self, tracker):
        # ------------ Connecting to wii remote.
        self.mote = None
        self.tracker = tracker
        self.monitor = xwiimote.monitor(True, True)
        while self.mote == None:
            self.mote = self.monitor.poll()
            rospy.sleep(1)


        # ------------ Setting up wii remote.
        self.mote = xwiimote.iface(self.mote)
        self.mote.open(self.mote.available() | xwiimote.IFACE_WRITABLE)
        self.mote.open(self.mote.available() | xwiimote.IFACE_ACCEL)
        self.mote.open(self.mote.available() | xwiimote.IFACE_MOTION_PLUS)
        self.poller = poll()
        self.poller.register(self.mote.get_fd(), POLLIN)
        self.event = xwiimote.event()

        # ------------ Parameters, data, etc.
        self.current = [0,0,0]
        self.resting = [0,0,0]
        self.last_reading = rospy.Time.now()
        self.target = [-70, 0, 10] # just for testing purposes
        self.index = 0 # ditto

        # ----------- ROS Publishers/subscribers.
        self.button_pub = rospy.Publisher("/wii_buttons", Int32, queue_size=10)
        self.orientation_pub = rospy.Publisher("/wii_orientation", )
        rospy.Subscriber("/wii_rumble", Bool, self.set_rumble)


    def run(self):
        self.poller.poll()
        try:
            self.mote.dispatch(self.event)
        except IOError:
            # print "IOError in run dispatch"
            pass

        if self.event.type == xwiimote.EVENT_MOTION_PLUS:
            self.handle_motion()
            self.check_if_close()

        elif self.event.type == xwiimote.EVENT_KEY:
            self.handle_buttons()
            (code, state) = self.event.get_key()
            if True: # right now using A and B buttons as a control thingie.
                if code == 4:
                    self.set_resting()
                elif code == 5:
                    self.set_zero()
            #Keys: 0 = left, 1: right, 2: up, 3 = down, 4 = A, 5 = B, 6 = +, 7 = -, 8 = home, 9 = 1, 10 = 2.




    def handle_motion(self):
        """
        handles motion plus events. basically, integrates the angular velocities into a current pose.
        then, publishes to the motion topic.
        still working on the roll thingie, the ros transforms I tried earlier didn't work quite right. probably gonna muck around with cosines &c tomorrow.
        """
        change = self.event.get_abs(0)
        now = rospy.Time.now()
        dt = now - self.last_reading
        self.last_reading = now

        self.current[0] += (change[0] * math.cos(math.radians(self.current[1])) + change[2] * math.sin(math.radians(self.current[1])))*(dt.nsecs/1000000000.0)*(1.492/20.0)**2
        self.current[2] += (change[2] * math.cos(math.radians(self.current[1])) + change[0] * math.sin(math.radians(self.current[1])))*(dt.nsecs/1000000000.0)*(1.492/20.0)**2
        self.current[1] += change[1]*(dt.nsecs/1000000000.0)*(1.492/20.0)**2

        a = numpy.array([int(i) for i in self.current], dtype=numpy.int32)
        self.orientation_pub.publish(a)

    def handle_buttons(self):
        (code, state) = self.event.get_key()
        self.button_pub.publish(code)

    def set_rumble(self, msg):
        try:
            self.mote.rumble(msg.data)
        except IOError:
            # print "Error setting rumble."
            pass


    def check_if_close(self):
        """
        if within some [pretty much arbitrary right now] distance of the target, rumbles. else, no rumble.
        """    
        # distance = math.sqrt((self.target[0]-self.current[0])**2 + (self.target[1]-self.current[1])**2 + (self.target[2]-self.current[2])**2)
        distance = math.sqrt((self.target[0]-self.current[0])**2 + (self.target[2]-self.current[2])**2)
        # print distance
        # rospy.sleep(.01) # frees up the wm again. can probably be a lot less time.
        try:
            if distance < 10:
                self.mote.rumble(True)
            else:
                self.mote.rumble(False)
        except IOError:
            # print "Error setting rumble."
            pass


    def set_zero(self):
        self.current = [0,0,0]

    def set_resting(self):
        self.index = 0
        drifts = []

        while self.index < 250: #...first handful of signals are always wacky; this gets it to ignore them.
            try:
                self.poller.poll()
                self.mote.dispatch(self.event)
                self.index += 1
            except IOError:
                print "ioerror in set_resting waiting period"

        while self.index < 500:
            try:
                self.poller.poll()
                self.mote.dispatch(self.event)
                reading = self.event.get_abs(0)
                if self.event.type == xwiimote.EVENT_MOTION_PLUS:
                    drifts.append(reading)
                    self.index += 1
            except IOError:
                print "ioerror in set_resting drift getting period"
        avg_reading = [(sum(i[j] for i in drifts)/len(drifts)) for j in range(3)]
        self.resting = avg_reading
        print "resting: \t", self.resting


if __name__ == "__main__":
    rospy.init_node("wm")
    wm = Wii_pointer("tracker goes here")
    start_time = rospy.Time.now()
    wm.set_resting()
    while not rospy.is_shutdown():
        wm.run()
        if rospy.Time.now() - start_time > rospy.Duration(180):
            print wm.index, '\t', wm.current
            exit()