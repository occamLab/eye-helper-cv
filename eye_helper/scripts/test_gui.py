"""
rough draft of a gui for rapidly switching between possible versions while codesigning.
"""
import rospy
import Tkinter as tk
import subprocess
from tango_tracker import Tango_tracker
from computer_speech import Speak_3d_coords

class Controller(tk.Frame):
    def __init__(self, module1, module2, master=None):
        tk.Frame.__init__(self, master)
        self.m1 = module1
        self.m2 = module2
        self.grid()
        self.createWidgets()

    def createWidgets(self):
        dc = 1.00
        self.m1_dc = tk.Scale(self, label="beep frequency coefficient", variable=dc, from_=0.1, to=1.0, resolution=0.1)
        self.m1_dc.grid()
        self.on_m1 = tk.Button(self, text="angle beeps on", command=self.m1.turn_on)
        self.on_m1.grid()
        self.off_m1 = tk.Button(self, text="angle beeps off", command=self.m1.turn_off)
        self.off_m1.grid()
        self.on_m2 = tk.Button(self, text='3d speech on', command=self.m2.turn_on)
        self.on_m2.grid()
        self.off_m2 = tk.Button(self, text='3d speech off', command=self.m2.turn_off)
        self.off_m2.grid()
        self.quit_button = tk.Button(self, text="Quit",command=self.quit)
        self.quit_button.grid()

    def call_all(self):
        self.m1.delay_coefficient = self.m1_dc.get()
        self.m1.call()
        self.m2.call()
        self.after(10, self.call_all)

class Version_one():
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
        self.base_filename = "height{}angle{}.wav"
        self.filename = ""
        self.sound_pub=rospy.Publisher('/sound_info', Sound, queue_size=10)
        self.mix=None

    def toggle(self):
        self.isOn = not self.isOn

    def turn_on(self):
        self.isOn = True

    def turn_off(self):
        self.isOn = False

    def call(self):
        if self.isOn:
            self.tracker.refresh_all()
            if self.tracker.xy_distance != None and self.tracker.angle_to_go != None:
                self.run()

    def run(self):
        delay = rospy.Duration(self.dist_to_delay(self.tracker, self.delay_coefficient))
        if rospy.Time.now() - self.last_tone < delay:
            return
        self.last_tone = rospy.Time.now()
        vol = self.angle_to_volume(self.tracker, self.volume_coefficient)
        atg = self.tracker.angle_to_go
        if atg >= 0:
            self.filename = "{}height{}angle{}.wav".format(self.path, 4, 90)
            self.mix=[100,0]
        else:
            self.filename = "{}height{}angle_{}.wav".format(self.path, 4, 90)
            self.mix=[0,100]

        self.sound_info= Sound(file_path=self.filename,volume=float(vol),mix_left=self.mix[0],mix_right=self.mix[1])
        self.sound_pub.publish(self.sound_info)
        self.play_audio(vol)
        

    def dist_to_delay(self, tracker, coefficient, cutoff=4, reverse=False):
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

class Version_two():
    """
    Forward and sideways distances. Not really sure how to communicate this, but anyways.
    Right this moment, left/right distance and side is communicated by frequency.
    """
    def __init__(self, tracker):
        self.tracker = tracker
        self.isOn = False
        self.volume_coefficient = 1.0 #thing to change.
        self.delay_coefficient = 0.5 #thing to change.
        self.last_played = rospy.Time.now()
        self.player = "aplay"
        self.path = "../../GeneratedSoundFiles/computer_speech/"
        self.base_filename = "{}.wav"
        self.filename = ""

    def toggle(self):
        self.isOn = not self.isOn

    def call(self):
        if self.isOn:
            self.tracker.refresh_all()
            if self.tracker.forward_distance != None and self.tracker.right_distance != None:
                self.run()

    def run(self):
        if (rospy.Time.now() - self.last_played) < rospy.Duration(8):
            return
        fd_signed = self.tracker.forward_distance
        fd = abs(fd_signed)
        rd_signed = self.tracker.right_distance
        rd = abs(rd_signed)
        values_to_play = [str(int(fd)), 'pt', str(int(10*((fd - int(fd)) - (fd - int(fd))%0.1)))]
        if fd_signed > 0:
            values_to_play.append('forward')
        else:
            values_to_play.append('back')
        values_to_play.append('and')
        values_to_play.extend([str(int(rd)),'pt',str(int(10*((rd-int(rd))-(rd-int(rd))%.1)))])
        if rd_signed > 0:
            values_to_play.append('right')
        else:
            values_to_play.append('left')
        for i in values_to_play:
            popen = subprocess.Popen('aplay {}{}.wav'.format(self.path,i), shell=True)
            popen.communicate()
            

if __name__ == "__main__":
    # tt = Tango_tracker()
    # v1 = Version_one(tt)
    # v2 = Version_two(tt)
    # control = Controller()
    # control.master.title("Testing GUI")
    # control.after(100, control.call_all)
    # control.mainloop()
# 
    tt = Tango_tracker()
    v1 = Version_one(tt)
    v2 = Speak_3d_coords(tt)
    control = Controller(v1, v2)
    control.master.title("Testing GUI")
    control.after(100, control.call_all)
    control.mainloop()
    # while True:
    #     tt.refresh_all()
    #     print tt.xy_distance
    #     if tt.xy_distance < 0.1 and tt.xy_distance != None:
    #         exit()
    #     if tt.xy_distance < 1:
    #         v1.isOn = False
    #         v2.isOn = True
    #     else:
    #         v1.isOn = True
    #         v2.isOn = False
    #     v1.call()
    #     v2.call()




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

# def dist_to_delay(tracker, coefficient, cutoff=4, reverse=False):
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