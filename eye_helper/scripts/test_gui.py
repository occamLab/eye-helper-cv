#!/usr/bin/env python
"""
rough draft of a gui for rapidly switching between possible versions while codesigning.
Currently imports the classes it uses from computer_speech.py and angle_distance.py.
"""
import rospy
import Tkinter as tk
import subprocess
from tango_tracker import Tango_tracker
from computer_speech import Speak_3d_coords
from angle_distance import Angle_and_distance, Offset_angle_and_distance

class Controller(tk.Frame):
    def __init__(self, module_list, master=None):
        tk.Frame.__init__(self, master)
        self.m = module_list
        self.grid()
        self.createWidgets()

    def createWidgets(self):
        dc = 1.00
        self.m1_dc = tk.Scale(self, label="beep frequency coefficient", variable=dc, from_=0.1, to=1.0, resolution=0.1, orient=tk.HORIZONTAL, )
        self.m1_dc.grid()
        self.m1_dc.set(0.5)
        self.on_m1 = tk.Button(self, text="angle beeps on", command=self.m[0].turn_on)
        self.on_m1.grid()
        self.off_m1 = tk.Button(self, text="angle beeps off", command=self.m[0].turn_off)
        self.off_m1.grid()
        self.on_m2 = tk.Button(self, text='3d speech on', command=self.m[1].turn_on)
        self.on_m2.grid()
        self.off_m2 = tk.Button(self, text='3d speech off', command=self.m[1].turn_off)
        self.off_m2.grid()
        self.quit_button = tk.Button(self, text="Quit",command=self.quit)
        self.quit_button.grid()

    def call_all(self):
        self.m[0].delay_coefficient = self.m1_dc.get()
        for i in self.m:
            i.call()
        self.after(10, self.call_all)



if __name__ == "__main__":

    tt = Tango_tracker()
    v1 = Angle_and_distance(tt)
    v2 = Speak_3d_coords(tt)
    control = Controller([v1, v2])
    control.master.title("Testing GUI")
    control.after(100, control.call_all)
    control.mainloop()
