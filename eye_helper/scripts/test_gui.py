"""
rough draft of a gui for rapidly switching between possible versions while codesigning.
"""

import rospy
# import rospkg
# from std_msgs.msg import Char
import Tkinter as tk
import subprocess
from tango_tracker import Tango_tracker

class Controller(tk.Frame):
    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        self.grid()
        self.createWidgets()

    def createWidgets(self):
        dc = 1.00
        self.v1_dc = tk.Scale(self, variable=dc, from_=0.1, to=1.0, resolution=0.1)
        self.v1_dc.grid()
        self.toggle_v1 = tk.Button(self, text="v1", command=v1.toggle)
        self.toggle_v1.grid()
        self.quit_button = tk.Button(self, text="Quit",command=self.quit)
        self.quit_button.grid()

    def call_all(self):
        v1.delay_coefficient = self.v1_dc.get()
        v1.call()
        self.after(10, self.call_all)

class version_one():
    """
    maps volume to amount that the angle is off, side to angle, and distance to frequency.
    """
    def __init__(self, tracker):
        self.tracker = tracker
        self.isOn = False
        self.volume_coefficient = 1.0 #thing to change.
        self.delay_coefficient = 0.5 #thing to change.
        self.last_tone = rospy.Time.now()
        self.player = "aplay"
        self.path = "../../GeneratedSoundFiles/"
        self.filename = ""

    def toggle(self):
        self.isOn = not self.isOn

    def call(self):
        if self.isOn:
            self.tracker.refresh_all()
            if self.tracker.xy_distance != None and self.tracker.angle_to_go != None:
                self.run()

    def run(self):
        delay = rospy.Duration(self.dist_to_freq(self.tracker, self.delay_coefficient))
        print '======================='
        print delay
        print self.last_tone
        print rospy.Time.now()
        print '-----------------------'
        if rospy.Time.now() - self.last_tone < delay:
            return
        self.last_tone = rospy.Time.now()
        vol = self.angle_to_volume(self.tracker, self.volume_coefficient)
        atg = self.tracker.angle_to_go
        if atg >= 0:
            self.filename = "{}height{}angle{}.wav".format(self.path, 4, 90)
        else:
            self.filename = "{}height{}angle_{}.wav".format(self.path, 4, 90)
        self.play_audio(vol)

    def dist_to_freq(self, tracker, coefficient, cutoff=4, reverse=False):
        """input the tracker and the coefficient. Takes the xy distance from the tracker, multiplies it by the coefficient, and returns the value - basically, the time to wait. For example, 2m -> 1m with coeff = 2 means 4s -> 2s delay."""
        xy_distance = tracker.xy_distance
        if xy_distance > cutoff:
            xy_distance = cutoff
        if not reverse:
            return xy_distance*coefficient
        else:
            return (cutoff * coefficient) - (xy_distance * coefficient)

    def angle_to_volume(self, tracker, coefficient, max_volume=40, reverse=False):
        atg = tracker.angle_to_go
        if not reverse:
            v = abs(atg*coefficient)
            if v > max_volume:
                v = max_volume
            return v
        else:
            v = max_volume - (coefficient*atg)
            if v < 0:
                v = 0
            elif v > max_volume:
                v = max_volume
            return v

    def play_audio(self, volume):
        cmd = 'amixer -D pulse sset Master {}%'.format(volume)
        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()
        cmd = "{} {}".format(self.player, self.filename)
        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()

if __name__ == "__main__":
    tt = Tango_tracker()
    v1 = version_one(tt)
    v1.isOn = True
    control = Controller()
    control.master.title("Testing GUI")
    control.after(100, control.call_all)
    control.mainloop()




# if __name__ == "__main__":
#     tt = Tango_tracker()
#     v1 = version_one(tt)





#NTS: look into ros parameters later

# class Player():
#     def __init__(self, tracker, runFunction):
#         self.tracker = tracker
#         self.isOn = False
#         self.run = runFunction

#     def call(self):
#         if self.isOn:
#             self.run(self.tracker)


# # tt = Tango_tracker()

# def dist_to_freq(tracker, coefficient, cutoff=4, reverse=False):
#     """input the tracker and the coefficient. Takes the xy distance from the tracker, multiplies it by the coefficient, and returns the value - basically, the time to wait. For example, 2m -> 1m with coeff = 2 means 4s -> 2s delay."""
#     xy_distance = tracker.xy_distance
#     if xy_distance > cutoff:
#         xy_distance = cutoff
#     if not reverse:
#         return xy_distance*coefficient
#     else:
#         return (cutoff * coefficient) - (xy_distance * coefficient)

# def angle_to_side(tracker):
#     if tracker.angle_to_go >= 0:
#         return 90
#     else:
#         return -90

# def angle_to_volume(tracker, coefficient, max_volume=40, reverse=False):
#     atg = tracker.angle_to_go
#     if not reverse:
#         v = abs(atg*coefficient)
#         if v > max_volume:
#             v = max_volume
#         return v
#     else:
#         v = max_volume - (coefficient*atg)
#         if v < 0:
#             v = 0
#         elif v > max_volume:
#             v = max_volume
#         return v

# def dynamic_mode(f_one, f_two, x, cutoff=1):
#     if x > cutoff:
#         return f_one
#     else:
#         return f_two

# def 