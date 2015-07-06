#!/usr/bin/env python
"""
just a gui for controlling the straight line thingie.
"""
from straight_line import Straight_line
from tango_tracker import Tango_tracker
import Tkinter as tk
# import rospy

class Controller(tk.Frame):
    def __init__(self, module, master=None):
        tk.Frame.__init__(self, master)
        self.module = module
        self.grid()
        self.createWidgets()


    def createWidgets(self):
        self.start = tk.Button(self, text = "start line", command = self.module.begin_line)
        self.start.grid()
        self.finish = tk.Button(self, text = "stop line", command = self.module.finish)
        self.finish.grid()

    def call_module(self):
        self.module.call()
        self.after(10, self.call_module)

if __name__ == "__main__":
    tt = Tango_tracker()
    sl = Straight_line(Tango_tracker)
    control = Controller(sl)
    # control.title("Straight line keeping thingie.")
    control.after(100, control.call_module)
    control.mainloop()