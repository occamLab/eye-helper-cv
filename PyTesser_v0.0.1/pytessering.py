from pytesser import *


#playing with the samples
im = Image.open('monitortext.tiff')
text = image_to_string(im)
print text

# image conversion and then image_to_string
# im = Image.open('monitortext.jpg')
# im.show()
# im.save('monitortext.tiff')