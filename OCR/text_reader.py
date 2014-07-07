from PIL import Image
from PIL import ImageEnhance
from tesseract import *
from urllib import urlretrieve
 
def get(link):
    urlretrieve(link,'temp.png')
 
get('http://www.site.com/captcha.php');
im = Image.open("temp.png")
nx, ny = im.size
im2 = im.resize((int(nx*5), int(ny*5)), Image.BICUBIC)
im2.save("temp2.png")
enh = ImageEnhance.Contrast(im)
enh.enhance(1.3).show("30% more contrast")
 
imgx = Image.open('temp2.png')
imgx = imgx.convert("RGBA")
pix = imgx.load()
for y in xrange(imgx.size[1]):
    for x in xrange(imgx.size[0]):
        if pix[x, y] != (0, 0, 0, 255):
            pix[x, y] = (255, 255, 255, 255)
imgx.save("bw.gif", "GIF")
original = Image.open('bw.gif')
bg = original.resize((116, 56), Image.NEAREST)
ext = ".tif"
bg.save("input-NEAREST" + ext)
image = Image.open('input-NEAREST.tif')
print image_to_string(image)