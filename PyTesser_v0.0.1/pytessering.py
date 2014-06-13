from pytesser import *
im = Image.open('phototest.tif')
text = image_to_string(im)
print text