eye-helper-cv
=============

Prototyping computer vision/crowdsourcing features for an assistive technology project 

##### Working with OpenCV 
- HSV object tracking (At first glance, it's not the best for multi-color objects)
- Contour detection (Seems promising, need to demystify the math a bit and test on grocery store items)

#### Working with some tesseract python wrappers
- [Google's PyTesser module](https://code.google.com/p/pytesser/w/list)
- [pyocr on Github](https://github.com/jflesch/pyocr)
- Also remember to 
    sudo apt-get install tesseract-ocr
- Results are lackluster without training things :|

#### Plan: training the tesseract to work well with reading food labels
- [Instructions](https://github.com/greenteawarrior/eye-helper-cv/blob/master/installing-tesseract-leptonica.md) for installing tesseract-ocr 3.03 and leptonica 1.70
