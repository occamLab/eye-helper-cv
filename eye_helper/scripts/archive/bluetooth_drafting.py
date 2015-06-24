"""
pybluez module, to do bluetooth stuff with python. 
just a draft now, probably not even going to add this to github.

Note: right now, getting some manner of security user warning. I ought to look into that after lunch.
"""

# import bluetooth

# target_name = "wiimote" #replace w/ actual name
# target_address = 12:34:56:78:90 #replace w/ actual number.

# devices_in_range = bluetooth.discover_devices()

# for i in devices_in_range:
    # print i, bluetooth.lookup_name(i)


# if target_address in nearby_devices:
#     print "connected"
# else:
#     print "device not found."


import cwiid
import rospy
import time

print "Connect the wii remote by pressing and holding 1 + 2 simultaneously."
try:
  wiimote = cwiid.Wiimote()
except RuntimeError:
    print "First attempt to connect failed. Trying again."
    try:
        wiimote = cwiid.Wiimote()
    except RuntimeError:
        print "Second attempt failed. Exiting."
        exit()
print "Connected."
wiimote.rpt_mode = cwiid.RPT_BTN

while not rospy.is_shutdown():
    # print wiimote.state
    # print cwiid.BTN_1
    if wiimote.state['buttons'] & cwiid.BTN_B and not wiimote.state['buttons'] & cwiid.BTN_A:
        print "B pressed."
    elif wiimote.state['buttons'] & cwiid.BTN_A and wiimote.state['buttons'] & cwiid.BTN_B:
        print "A and B pressed."
    elif wiimote.state['buttons'] & cwiid.BTN_A:
        print "A pressed."
    time.sleep(0.01)
    # if cwiid.BTN_1 in wiimote.state['buttons']:
        # print "button 1 pressed. This is where the voice thing would play, in the more integrated thingie."


