#!/usr/bin/env Python
"""
Testing out the range for the rumble - like, minimum, full, the resolution/granularity, etc.
"""

import rospy
import xwiimote
import math
import time
from select import poll, POLLIN


class mote():
    def __init__(self):
        self.mote = None
        self.monitor = xwiimote.monitor(True, True)
        while self.mote == None:
            self.mote = self.monitor.poll()
            rospy.sleep(1)

        self.mote = xwiimote.iface(self.mote)
        self.mote.open(self.mote.available() | xwiimote.IFACE_WRITABLE)
        self.poller = poll()
        self.poller.register(self.mote.get_fd(), POLLIN)
        self.event = xwiimote.event()

    def rumble(self, proportion, time=2):
        r = rospy.Rate(1000)
        now = rospy.Time.now()
        length = rospy.Duration(time)
        c = .02
        while rospy.Time.now() - now < length:
            if proportion == 0:
                self.mote.rumble(False)
                rospy.sleep(c)
            elif proportion == 1:
                self.mote.rumble(True)
                rospy.sleep(c)
            else:
                self.mote.rumble(True)
                # r.sleep()
                rospy.sleep(c*proportion)
                self.mote.rumble(False)
                rospy.sleep(c*(1-proportion))
                # r.sleep()


if __name__ == "__main__":
    rospy.init_node("wm")
    wm = mote()
    props = [0]
    props.extend([.02*i for i in range(1,50)])
    props.append(1)
    for p in props:
        print p, "\n--------------"
        # wm.rumble(p)
    print "full"    
    wm.mote.rumble(True)
    rospy.sleep(3)
    print "off"
    wm.mote.rumble(False)