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