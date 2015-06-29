#!/usr/bin/env python
"""
rough draft of a gui for rapidly switching between possible versions while codesigning.
Currently imports the classes it uses from computer_speech.py and angle_distance.py.
"""
import rospy
import Tkinter as tk
import math
import subprocess
from tango_tracker import Tango_tracker
from height_communication_testing import Absolute_height, Angle_height, Body_mapping

class Controller(tk.Frame):
    def __init__(self, module_list, mote=None, master=None):
        tk.Frame.__init__(self, master)
        self.m = module_list
        self.grid()
        self.createWidgets()

    def createWidgets(self):
        dc = 1.00
        self.on_m1 = tk.Button(self, text="angle on", command=self.m[0].turn_on)
        self.on_m1.grid()
        self.off_m1 = tk.Button(self, text="angle off", command=self.m[0].turn_off)
        self.off_m1.grid()
        self.on_m2 = tk.Button(self, text='inches on', command=self.m[1].turn_on)
        self.on_m2.grid()
        self.off_m2 = tk.Button(self, text='inches off', command=self.m[1].turn_off)
        self.off_m2.grid()
        self.off_m3= tk.Button(self, text='body parts on', command=self.m[2].turn_on)
        self.off_m3.grid()
        self.off_m3 = tk.Button(self, text='body parts off', command=self.m[2].turn_off)
        self.off_m3.grid()
        self.quit_button = tk.Button(self, text="Quit",command=self.quit)
        self.quit_button.grid()

    def call_all(self):
        # self.m[0].delay_coefficient = self.m1_dc.get()

        for i in self.m:
            i.call()

        self.after(10, self.call_all)



if __name__ == "__main__":

    tt = Tango_tracker()
    v1 = Angle_height(tt)
    v2 = Absolute_height(tt)
    v3 = Body_mapping(tt)

    control = Controller([v1, v2, v3])
    control.master.title("Testing GUI")
    control.after(100, control.call_all)
    control.mainloop()
