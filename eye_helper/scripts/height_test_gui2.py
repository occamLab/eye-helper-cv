#!/usr/bin/env python

import rospy
import Tkinter as tk
import math
import subprocess
from tango_tracker import Tango_tracker
from computer_speech import Speak_3d_coords
from computer_speech2 import Speak_3d_directions
from angle_distance import Angle_and_distance, Offset_angle_and_distance
from integrated_close_height_angle_test import Reach
from Body_mapping_and_beeps import Angle_body_mapping

class Body_map_controller(tk.Frame):
    """ 
    GUI for height testing with: 1) Body mapping integrated with angle computer speech and 2) Body mapping integrated with angle beeps. 
    """

    def __init__(self, module_list=[Reach(Tango_tracker()), Angle_body_mapping(Tango_tracker())], master=None):
        tk.Frame.__init__(self, master)
        self.m = module_list
        self.grid()
        self.createWidgets()

    def createWidgets(self):
    	dc = 1.00
        self.m1_dc = tk.Scale(self, label="beep frequency coefficient", variable=dc, from_=0.1, to=1.0, resolution=0.1, orient=tk.HORIZONTAL, )
        self.m1_dc.grid()
        self.m1_dc.set(0.5)

        self.tango_height_entry = tk.Entry(self)
        self.tango_height_entry.grid(column=0, row=1)
        self.person_height_entry = tk.Entry(self)
        self.person_height_entry.grid(column=2, row=2)
        self.eye_entry = tk.Entry(self)
        self.eye_entry.grid(column=0, row=2)
        self.shoulder_entry = tk.Entry(self)
        self.shoulder_entry.grid(column=0, row=3)
        self.elbow_entry = tk.Entry(self)
        self.elbow_entry.grid(column=0, row=4)
        self.hip_entry = tk.Entry(self)
        self.hip_entry.grid(column=0, row=5)
        self.knee_entry = tk.Entry(self)
        self.knee_entry.grid(column=2, row=1)

        x=[0,1]

        self.tango_height_send = tk.Button(self, text = "set tango height", command = lambda: self.m[0].set_tango_height(float(self.tango_height_entry.get())) and self.m[1].set_tango_height(float(self.tango_height_entry.get()))) 
        self.tango_height_send.grid(column=1, row=1)
        self.person_height_send = tk.Button(self, text = "set person height", command = lambda: self.m[0].set_person_height(float(self.person_height_entry.get())) and self.m[1].set_person_height(float(self.person_height_entry.get())))
        self.person_height_send.grid(column=3, row=2)
        self.eye_send = tk.Button(self, text = "set eye height", command = lambda: self.m[0].set_part("eye", float(self.eye_entry.get())) and self.m[1].set_part("eye", float(self.eye_entry.get())))
        self.eye_send.grid(column=1, row=2)
        self.shoulder_send = tk.Button(self, text = "set shoulder height", command = lambda: self.m[0].set_part("shoulder", float(self.shoulder_entry.get())) and self.m[1].set_part("shoulder", float(self.shoulder_entry.get())))
        self.shoulder_send.grid(column=1, row=3)
        self.elbow_send = tk.Button(self, text = "set elbow height", command = lambda: self.m[0].set_part("elbow", float(self.elbow_entry.get())) and self.m[1].set_part("elbow", float(self.elbow_entry.get())))
        self.elbow_send.grid(column=1, row=4)
        self.hip_send = tk.Button(self, text = "set hip height", command = lambda: self.m[0].set_part("hip", float(self.hip_entry.get())) and self.m[1].set_part("hip", float(self.hip_entry.get())))
        self.hip_send.grid(column=1, row=5)
        self.knee_send = tk.Button(self, text = "set knee height", command = lambda: self.m[0].set_part("knee", float(self.knee_entry.get())) and self.m[1].set_part("knee", float(self.knee_entry.get())))
        self.knee_send.grid(column=3, row=1)

        self.on_m1 = tk.Button(self, text = "speech on", command = self.m[0].turn_on)
        self.on_m1.grid(column=2, row=4)
        self.off_m1 = tk.Button(self, text = "speech off", command = self.m[0].turn_off)
        self.off_m1.grid(column=2, row=5)
        self.on_m2 = tk.Button(self, text = "beep on", command = self.m[1].turn_on)
        self.on_m2.grid(column=3, row=4)
        self.off_m2 = tk.Button(self, text = "beep off", command = self.m[1].turn_off)
        self.off_m2.grid(column=3, row=5)

        # self.turn_off.grid(column=0, row=6)
        # self.turn_on.grid(column=1, row=6)

    def call_all(self):
    	self.m[1].delay_coefficient = self.m1_dc.get()
        for i in self.m:
            i.call()

        self.after(10, self.call_all)

if __name__ == "__main__":

    tt = Tango_tracker()
    v1 = Reach(tt)
    v2 = Angle_body_mapping(tt)

    control = Body_map_controller()
    control.master.title("Testing GUI")
    control.after(100, control.call_all)
    control.mainloop()