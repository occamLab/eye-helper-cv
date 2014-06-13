import pyocr

im = Image.open('adobeCover.tiff')
text = image_to_string(im)
print text