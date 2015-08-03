eye-helper-cv
=============

- Prototyping computer vision/crowdsourcing features for an assistive technology project 
- Current quest: Object tracking (moving camera, static grocery store items)
- Updates: android implementation in progress (see the [https://github.com/cypressf/eye-helper](eye-helper-audio repository))
- Updates: python implementation should be used for optimizing object tracking technique (how many times SIFT needs to run, etc) but eventually we'll all be working on the android app
- Plan: For concept testing/proof of concept work, we're going to run the python code on the comprobo Raspberry pi setup. The scripts of interest for this should be live_meanshift.py, object_match.py, and compare_kpd.py

###Repository contents
- gstore_csv (contains the ground truth information for sub-snippets of the datasets in gstore-snippets)
- gstore_snippets (contains video frames for the cookie, cereal, catfood, and pasta datasets)
- OT (contains python scripts for object tracking, keypoint/descriptor methods, data extraction...)
- OT_res (contains results for things generated with OT scripts)
- calib_pics (a handful of images used for camera calibration in calibrate_from_chessboard() from OT/epipolar.py)
- preliminary statistics adventures (our first stab at stats/quantitative validation for results with the cookie dataset)

###Stuff the Tango code uses:

- GeneratedSoundFiles
  - Contents: Computer-generated speech and tone files, for other scripts to play. We have tried text-to-speech libraries; they were both slow and unclear.
- eye_helper
  - msg
    -Contents: Definition for several custom ROS messages, which can broadcast information on what sounds and speech messages have played (or should be played).
  - scripts
    -Contents: An archive of old code, and a heckuva lot of scripts (see below).

Info on scripts:
================

- file (key ones in **bold**; a lot of the others are mainly for testing/tweaking the key ones.)
  - description
    - sub-scripts, classes, etc.
  - dependencies

- **angle_distance.py**
  - Contains two classes for a beep/tone-based navigation system. 
    - Angle_and_distance: beeps with a side & volume corresponding to the angle of the target, and a speed corresponding to distance. 
    - Offset_angle_and_distance: does the same - but instead of directing to a target, it directs to a point a certain distance "offset" from the target (for example: directing to something .3 meters in front of the object).
  - Needs: a "tracker" (e.g. tango_tracker.py), the Sound message from `/../msg`, ROS, builtins.

- beeps_forwardright.py
  - TODO
  - [unnecessary? not sure. nts: ask someone. i think this is a duplicate-y thing of angle_distance?]

- Body_mapping_and_beeps.py
  - If the user is further than (an arbitrary distance) from the target, or is not facing towards the target, it uses the angle-and-distance system to direct them. Otherwise, it uses computer speech to tell them where to reach, by "body mapping" - giving directions in terms of human proportions.
  - Needs: a "tracker", the Sound and Speech messages from `/../msg`, ROS, builtins.

- **breadcrumb.py**
  - Provides the Breadcrumb_tracker object - an object which keeps up-to-date information on the tango & target locations for other scripts to reference. This tracker sets the target by having the user set "landmarks" or a "trail" (a-la Hansel and Gretel), rather than by having a sighted person select an object from an image.
  - Needs: ROS, builtins.

- **computer_speech2.py**
  - TODO

- computer_speech_angle.py
  - TODO
  - nts: pretty sure that its height isn't what it's supposed to be?

- computer_speech_inches.py
  - A version of computer_speech (see below) which gives information in terms of inches instead of meters.

- computer_speech.py
  - Contains two classes which use computer-generated speech (rather than tones) to guide the user.
    - Orthogonal_distances: Tells the user how far forward/back, and right/left, to go. Not ideal if the user's angle is far off.
    - Speak_3d_coords: Tells the user the target's location, in terns of how far forward/back, right/left, and up/down from the Tango. Note: not the most responsive time-wise. Also, most people **hate** this one.
      - Like, really hate. **Haaaate** hate.
  - Needs: a "tracker", the Speech message from `/../msg`, ROS, builtins.

- **height_communication_testing.py**
  - A smattering of different ways to communicate height information.
    - Absolute_height: gives the distance above/below the Tango in either metric or imperial, depending on settings.
    - Angle_height: gives the vertical angle - the pitch - from the tango to the target.
    - Body_mapping: Gives the height of the target in (customizeable) human heights, e.g. "half a foot below shoulder level."
    -Body_map_controller, which allows for on-the-fly adjustment of things like the user's height, the tango height, etc.
  - Needs: a "tracker", the Speech message from `/../msg`, TKinter, ROS, builtins.

- height_test_gui2.py
  - Starts a (muted/turned-off) version of a bunch of different objects, like the computer speech, angle_distance, and height testing classes. Creates a window (with buttons/sliders/text entry) to turn them on/off on-the-fly.
  - Needs: pretty much everything that anything else needs.

- height_test_gui.py
  - Like above, but just for height (only height info; no direction, distance, etc navigation).
  - Needs: ditto.

- **landmark.py**
  - uses wii to save landmarks and
    - A -> initiate landmark 
    - 1 -> add position to trail 
    - up/down -> select landmark to be published 
    - home -> publish the trail of the elected landmark
  - tracker: Tango_tracker
  - uses computer speech to guide back. 

- point_queue.py
  - TODO

- ransac.py
  - A point-cloud-y / tango-y implementation of a RANSAC algorithm ([https://en.wikipedia.org/wiki/RANSAC]).
  - Used to get the plane of the target object - a line more-or-less on par with the shelf, box, etc., so that something can be offset parallel or perpendicular to it.
  - Not very precise, unfortunately. Better than nothing, but there's more variation than one would like.
  - Needs: builtins.

- rumble.py
  - kind of an example for how to rumble the wii remote at different intensities.
  - also unnecessary; will get moved to archive soon.
  - Needs: xwiimote, ROS, builtins.

- sound_listener.py
  - Listens to the `/sound_info` ROS topic, which publishes Sound messages. Then, plays the appropriate sound file.
  - Needs: the Sound message from `/../msg`, ROS, builtins.

- speech_listener.py
  - Listens to the `/speech_info` ROS topic, which publishes Speech messages. Then, plays the appropriate speech file.
  - Needs: the Speech message from `/../msg`, ROS, builtins.

- straight_line_gui.py
  - Just some buttons to click for the straight_line file.
  - Needs: straight_line, a "tracker", TKinter.

- straight_line.py
  - Based on user comments on moving in straight lines, especially early on. This provides audio feedback if one strays too far from a set line; more off means louder feedback.
  - Needs: a "tracker", ROS, builtins.

- tango_pose_calc2.py
  - TODO

- tango_tracker.py
  - Keeps track of the location of the tango, the target, the distance / angle between them, the user's orientation, &c., in a way that can be referenced by other scripts.
  - AKA listens to a bunch of ROS topics and collates the info so that other scripts don't have to.
  - Needs: ROS, builtins, (ransac).

- test_gui.py
  - An early GUI for user tests, so that different systems and mappings can be tried out without having to restart all of the programs.
  - Needs: a "tracker", associated files/guides, TKinter, builtins.

- wii_pointer_draft.py
  - Gets info from a Wii remote (connect it from terminal first, via bluetooth). Integrates the angular velocity info to get its orientation; reads buttons, and can make the remote rumble. Sends out info through ROS topics.
  - Needs: xwiimote, ROS, builtins, (a "tracker").


Note: the TODOs mean that the writeup/documentation-y stuff is todo, not that the file itself is todo.


FAQ:
==============
- Q: Why the `#!/usr/bin/env` python at the top?
  - A: This allows python files marked as executable to be run from anywhere via the rosrun command - it tells ROS what program to use to run the file.
- Q: What about user tests?
  - A: For security, convenience, &c., the user test information and notes is on a (private) Google Drive. Same goes for bag files.
- Q: What's body mapping?
  - A: There's probably a legit term for it, but I don't know it. It's giving instructions/info in body-relative terms: instead of "1.3 meters above the ground", "at eye-level" etc.
- Q: Why are there terrible puns in a bunch of the in-file comments?
  - A: There aren't. They are *excellent* puns.
