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
  - Contents: Computer-generated speech and tone files, for other scripts to play.
- eye_helper
  - msg
    -Contents: Definition for several custom ROS messages, which can broadcast information on what sounds and speech messages have played (or should be played).
  - scripts
    -Contents: An archive of old code, and a heckuva lot of scripts (see below).

Info on scripts:
================

- file
  - description
    - sub-scripts, classes, etc.
  - dependencies

- angle_distance.py
  - Contains two classes for a beep/tone-based navigation system. 
    - Angle_and_distance: beeps with a side & volume corresponding to the angle of the target, and a speed corresponding to distance. 
    - Offset_angle_and_distance: does the same - but instead of directing to a target, it directs to a point a certain distance "offset" from the target (for example: directing to something .3 meters in front of the object).
  - Needs: a "tracker" (e.g. tango_tracker.py), the Sound message from `/../msg`, builtins.

- beeps_forwardright.py
  - TODO
  - [unnecessary? not sure. nts: ask someone. i think this is a duplicate-y thing of angle_distance?]

- Body_mapping_and_beeps.py
  - If the user is further than (an arbitrary distance) from the target, or is not facing towards the target, it uses the angle-and-distance system to direct them. Otherwise, it uses computer speech to tell them where to reach, by "body mapping" - giving directions in terms of human proportions.
  - Needs: a "tracker", the Sound and Speech messages from `/../msg`, builtins.

- breadcrumb.py
  - Provides the Breadcrumb_tracker object - an object which keeps up-to-date information on the tango & target locations for other scripts to reference. This tracker sets the target by having the user set "landmarks" or a "trail" (a-la Hansel and Gretel), rather than by having a sighted person select an object from an image.
  - Needs: builtins.

- computer_speech2.py
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
  - Needs: a "tracker", the Speech message from `/../msg`, builtins.

- height_communication_testing.py
  - 








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
