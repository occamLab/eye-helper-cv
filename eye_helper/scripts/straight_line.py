"""
p1 & p2s' suggestions about straight lines, adventitious blindness, etc.

Is it more useful to have it correct angle (pre-emptively), or for absolute distance left/right?
"""

from tango_tracker import Tango_tracker # this import might not be necessary in the future, but it's convenient for testing purposes.
import math
import rospy
import rospkg
import subprocess
# import cwiid



class Straight_line():
    """
    directs to/keeps on a straight line, e.g. for a crosswalk or sidewalk.
    """

    def __init__(self, tracker):
        """
        tracker = tango tracker.
        """
        self.tracker = tracker
        self.isOn = False
        self.isTracking = False
        self.point = None
        self.direction = None #looks only at XY
        self.isAvoid = True # this parameter is for whether the beeps direct you to, or from, stuff. if True, then a beep from the left means you are *too far left* and should not go further, kinda like car-parking style.
        self.distance_threshold = 0.3048 # distance from line before the warning kicks in. .3048 m = 1 foot.
        self.angle_threshold = 60 # degrees before the warning kicks in. At present, no angle warnings are being used.
        self.last_played = rospy.Time.now()
        self.delay_coefficient = 1
        self.rospack = rospkg.RosPack()
        self.path = self.rospack.get_path('eye_helper') + '/../GeneratedSoundFiles/'
        self.filename = "height4angle5.wav"

    def toggle(self):
        self.isOn = not self.isOn

    def turn_on(self):
        self.isOn = True

    def turn_off(self):
        self.isOn = False

    def begin_line(self):
        """
        When called, makes note of current location and orientation.
        """
        self.point = (self.tracker.x, self.tracker.y, self.tracker.z)
        #probably don't need the z-coord actually.
        self.direction = self.tracker.yaw

    def finish(self):
        self.point = None
        self.direction = None

    def get_distance_to_line(self):
    """
    it's probably easiest + most robust to just grab another arbitrary colinear point, with cos+sin and the yaw + starting point, and then calculate distance to that two-points-line?
    """
        # if self.point == None or self.direction == None:
            # return # already checked prior to this running.
        x = self.tracker.x
        y = self.tracker.y
        x_0 = self.point[0]
        x_1 = self.point[0] + math.cos(self.direction)
        y_0 = self.point[1]
        y_1 = self.point[1] + math.sin(self.direction)
        d = ((y_1 - y_0)*x - (x_1 - x_0)*y + x_1*y_0 - y_1*x_0) / math.sqrt( (y_1 - y_0)**2 + (x_1 - x_0)**2 )
        return d # if d is positive, i think that means to the right; negative means to the left? I might be completely wrong on that, though.

    def get_angle_to_line(self):
        """
        gets the absolute difference between current angle facing and the original one from begin_line. 
        this function isn't super useful on its own - mostly just naming it to hopefully make the main function more readable.
        """
        return math.degrees(self.tracker.yaw - self.direction) #should this be coerced to +- pi?

    def call(self):
        if self.isOn:
            self.tracker.refresh_all()
            if self.point != None and self.direction != None:
                self.run()

    def run(self):
        d = self.get_distance_to_line()
        a = self.get_angle_to_line()
        #right now, angle isn't being used for anything in particular.
        if abs(d) > self.distance_threshold:

            delay = rospy.Duration(1.0 - min(1.0, self.delay_coefficient*(abs(d) - self.distance_threshold)))
            if rospy.Time.now - self.last_played < delay:
                return
            if d>0:
                ratio = [0,1]
            else:
                ratio = [1,0]
            self.play_audio(ratio)

    def play_audio(self, ratio):
        cmd = 'amixer -D pulse sset Master {}%,{}%'.format(30*ratio[0], 30*ratio[1])
        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()
        cmd = "aplay{}{}".format(self.path, self.filename)
        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()




if __name__ == "__main__":
    tt = Tango_tracker()
    sl = Straight_line(tt)
    sl.turn_on()
    rospy.sleep(5.0)
    sl.begin_line()
    while not rospy.is_shutdown():
        sl.call()