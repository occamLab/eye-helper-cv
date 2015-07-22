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
        self.target = [-40, 0, 20] # just for testing purposes
        self.index = 0 # ditto
        self.autoCheck = True

        # ----------- ROS Publishers/subscribers.
        self.button_pub = rospy.Publisher("/wii_buttons", Int32, queue_size=10)
        self.orientation_pub = rospy.Publisher("/wii_orientation", Int32MultiArray, queue_size=10)
        rospy.Subscriber("/wii_rumble", Bool, self.set_rumble)


    def run(self):
        self.tracker.refresh_all()
        self.get_target_from_tracker()
        self.poller.poll()
        try:
            self.mote.dispatch(self.event)
        except IOError:
            pass

        if self.event.type == xwiimote.EVENT_MOTION_PLUS:
            self.handle_motion()
            if self.autoCheck:
                self.check_if_close()

        elif self.event.type == xwiimote.EVENT_KEY:
            self.handle_buttons()
            (code, state) = self.event.get_key()
            if True: # right now using A and B buttons as a control thingie.
                if code == 5:
                    self.set_resting()
                    self.set_zero()
            #Keys: 0 = left, 1: right, 2: up, 3 = down, 4 = A, 5 = B, 6 = +, 7 = -, 8 = home, 9 = 1, 10 = 2.


    def get_target_from_tracker(self):
        """
        sets self.target based on values from self.tracker.
        """
        if self.tracker.z_distance == None or self.tracker.xy_distance == None or self.tracker.angle_to_go == None:
            return
        pitch = math.atan2(self.tracker.z_distance, self.tracker.xy_distance)
        yaw = self.tracker.angle_to_go # TODO: incorporate offset.
        self.target = [-1*yaw, 0, math.degrees(pitch)]


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

        a = np.array([int(i) for i in self.current], dtype=np.int32)
        b = Int32MultiArray(data=a)
        self.orientation_pub.publish(b)

    def handle_buttons(self):
        (code, state) = self.event.get_key()
        self.button_pub.publish(code)

    def set_rumble(self, msg):
        try:
            self.mote.rumble(msg.data)
        except IOError:
            pass


    def check_if_close(self):
        """
        if within some [pretty much arbitrary right now] distance of the target, rumbles. else, no rumble.
        """    
        distance = math.sqrt((self.target[0]-self.current[0])**2 + (self.target[2]-self.current[2])**2)
        print self.target
        try:
            # return
            if distance < 10:
                self.mote.rumble(True)
            else:
                self.mote.rumble(False)
        except IOError:
            pass


    def set_zero(self):
        self.current = [0,0,0]

    def set_resting(self):
        self.index = 0
        drifts = []

        while self.index < 400: #...first handful of signals are always wacky; this gets it to ignore them.
            try:
                self.poller.poll()
                self.mote.dispatch(self.event)
                self.index += 1
            except IOError:
                pass

        while self.index < 800:
            try:
                self.poller.poll()
                self.mote.dispatch(self.event)
                reading = self.event.get_abs(0)
                if self.event.type == xwiimote.EVENT_MOTION_PLUS:
                    drifts.append(reading)
                    self.index += 1
            except IOError:
                pass
        avg_reading = [(sum(i[j] for i in drifts)/len(drifts)) for j in range(3)]
        self.resting = avg_reading
        print "resting: \t", self.resting


if __name__ == "__main__":
    tt = Tango_tracker()
    wm = Wii_pointer(tt)
    start_time = rospy.Time.now()
    wm.set_resting()
    while not rospy.is_shutdown():
        wm.run()
        if rospy.Time.now() - start_time > rospy.Duration(1200):
            print wm.index, '\t', wm.current
            exit()