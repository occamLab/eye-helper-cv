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
        # self.mote.open(self.mote.available() | xwiimote.IFACE_ACCEL)
        self.mote.open(self.mote.available() | xwiimote.IFACE_MOTION_PLUS)
        self.poller = poll()
        self.poller.register(self.mote.get_fd(), POLLIN)
        self.event = xwiimote.event()
        self.current = [0,0,0]
        self.last_reading = rospy.Time.now()
        self.maybe_angle = [0,0,0]
        self.resting = [0,0,0]
        self.target = [-1000000, 0, 0]
        self.index = 0
        # self.zero = [0,0,0]


    def run(self):
        self.poller.poll()
        self.mote.dispatch(self.event)
        if self.event.type == xwiimote.EVENT_MOTION_PLUS:
            change = self.event.get_abs(0)
            # if self.resting[0] == 0:
                # self.resting = change
                # print "calibrated"
                # print self.resting
                # print "=============="
            # print change
            for i in range(3):
                self.current[i] += (change[i] - self.resting[i])
            self.index += 1
            if self.index%100 == 0:
                # print self.current, '\t \t || \t \t', change, "\t \t || \t \t", self.resting
                pass
            self.check_if_close()

    def check_if_close(self):
        """
        if within some [pretty much arbitrary right now] distance of the target, rumbles. else, no rumble.
        """    
        # self.poller.poll()
        # self.mote.dispatch(self.event)
        distance = math.sqrt((self.target[0]-self.current[0])**2 + (self.target[1]-self.current[1])**2 + (self.target[2]-self.current[2]))
        print distance
        rospy.sleep(.01)
        if distance < 200000:
            self.mote.rumble(True)
        else:
            self.mote.rumble(False)
        # pass


    def set_zero(self):
        self.current = [0,0,0]

    def set_resting(self):
        self.index = 0
        drifts = []

        while self.index < 500: #...first handful of signals are always wacky; this gets it to ignore them.
            self.poller.poll()
            self.mote.dispatch(self.event)
            self.index += 1

        while self.index < 1000:
            self.poller.poll()
            self.mote.dispatch(self.event)
            reading = self.event.get_abs(0)
            if self.event.type == xwiimote.EVENT_MOTION_PLUS:
                drifts.append(reading)
                # print "reading: \t", reading
                self.index += 1
        avg_reading = [(sum(i[j] for i in drifts)/len(drifts)) for j in range(3)]
        self.resting = avg_reading
        print "resting: \t", self.resting
        # yet = False
        # while yet == False:
        #     if self.event.type == xwiimote.EVENT_MOTION_PLUS:
        #         print self.event.get_abs(0)
        #         yet = True



# if __name__ == "__main__":
#     wm = Wii_pointer("tracker goes here")
#     wm.mote.rumble(True)
#     time.sleep(2)
#     wm.mote.rumble(False)


if __name__ == "__main__":
    rospy.init_node("wm")
    wm = Wii_pointer("tracker goes here")
    start_time = rospy.Time.now()
    wm.set_resting()
    while not rospy.is_shutdown():
        wm.run()
        if rospy.Time.now() - start_time > rospy.Duration(80):
            print wm.index, '\t', wm.current
            exit()