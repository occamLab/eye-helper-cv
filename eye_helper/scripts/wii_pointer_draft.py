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
# from tango_tracker import Tango_tracker


'''
class Wii_pointer():
    def __init__(self, tracker, address=None):
        self.tracker = tracker
        self.zeros = None
        self.shoulder_right_offset = 0 #for left, make negative.
        self.shoulder_up_offset = 0 
        # self.pair()
        if address == None: # for some reason, the wm+ doesn't like auto-connecting. can use hcitools to scan for the wm, then get its bluetooth address directly.
            print "press and hold 1 and 2."
            try:
                wm = cwiid.Wiimote()
            except RuntimeError:
                print "connection failed, try again."
                try:
                    wm = cwiid.Wiimote()
                except RuntimeError:
                    print "connection failed, terminating."
                    return
            self.wm = wm
        self.wm.rpt_mode = cwiid.RPT_ACC
        self.isZeroed = False


    def pair(self):
        print "press and hold 1 and 2."
        try:
            wm = cwiid.Wiimote()
        except RuntimeError:
            print "connection failed, try again."
            try:
                wm = cwiid.Wiimote()
            except RuntimeError:
                print "connection failed, terminating."
                return
        self.wm = wm


    def set_zero(self):
        "set current orientation to straight up"
        curr = self.wm.state['acc'] # (a, b, c) tuple; ints. a is the x-axis? I think? NTS: Try out just printing them out while connected.

        self.isZeroed = True
        pass
'''


class Wii_pointer():
    def __init__(self, tracker):
        self.mote = None
        self.tracker = tracker
        self.monitor = xwiimote.monitor(True, True)
        while self.mote == None:
            self.mote = self.monitor.poll()
            rospy.sleep(1)
        self.mote = xwiimote.iface(self.mote)

        self.mote.open(self.mote.available() | xwiimote.IFACE_WRITABLE)
        self.mote.open(self.mote.available() | xwiimote.IFACE_ACCEL)
        self.mote.open(self.mote.available() | xwiimote.IFACE_MOTION_PLUS)
        self.poller = poll()
        self.poller.register(self.mote.get_fd(), POLLIN)
        self.event = xwiimote.event()
        self.current = [0,0,0]
        self.last_reading = rospy.Time.now()
        self.maybe_angle = [0,0,0]
        self.resting = [0,0,0]
        self.target = [-70, 0, 10]
        self.index = 0


    def run(self):
        self.poller.poll()
        try:
            self.mote.dispatch(self.event)
        except IOError:
            # print "IOError in run dispatch"
            pass
        if self.event.type == xwiimote.EVENT_MOTION_PLUS:
            change = self.event.get_abs(0)
            now = rospy.Time.now()
            dt = now - self.last_reading
            self.last_reading = now
            for i in range(3):
                self.current[i] += (change[i] - self.resting[i])*(dt.nsecs/1000000000.0)*(1.492/20.0)**2 #mucking around to degrees
            self.index += 1
            if self.index%100 == 0:
                print [int(i) for i in self.current], '\t \t || \t \t', change, "\t \t || \t \t", dt.nsecs/1000000000.0
                pass
            self.check_if_close()
        elif self.event.type == xwiimote.EVENT_KEY:
            (code, state) = self.event.get_key()
            if state == 1 or True:
                if code == 4:
                    self.set_resting()
                elif code == 5:
                    self.set_zero()
            #Keys: 0 = left, 1: right, 2: up, 3 = down, 4 = A, 5 = B, 6 = +, 7 = -, 8 = home, 9 = 1, 10 = 2.
            print code, '\t || \t', state

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
            "ioerror in check rumble thingie."


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