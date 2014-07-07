eye-helper-cv
=============

Prototyping computer vision/crowdsourcing features for an assistive technology project 

### Object Tracking (current priority)

#### /preliminary-object-tracking
##### Tried different things in OpenCV 
- HSV object tracking (At first glance, it's not the best for multi-color objects)
- Contour detection (Seems promising, need to demystify the math a bit and test on grocery store items)

#### Feature matching/tracking with SIFT
##### Scripts of interest in top-level of this repo:
- label-data.py (allows human to identify object of interest in frames and create image set)
- find-truth.py (finds all keypoints in image set)
- keypoint-matching.py (matches keypoints between frames of interest) 

##### See results in /object-tracking-results
- cookie (with a blurry-ish training image taken from before the query frame in the video progression, and also using a nice, focused training image to compare results)

### /OCR 
####(Recontinue after object tracking is up and running)

#### Investigated some tesseract python wrappers
- [Google's PyTesser module](https://code.google.com/p/pytesser/w/list)
- [pyocr on Github](https://github.com/jflesch/pyocr)
- Also remember to 
    sudo apt-get install tesseract-ocr
- Results are lackluster without training things :|

#### Plan: training the tesseract to work well with reading food labels
- [Instructions](https://github.com/greenteawarrior/eye-helper-cv/blob/master/installing-tesseract-leptonica.md) for installing tesseract-ocr 3.03 and leptonica 1.70
