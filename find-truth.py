import cv2
import numpy as np 
import csv

"""
Takes in an image set and detector. For each frame in the image,
it'll find all of the keypoints within the object of interest
(i.e. whatever was selected when labeling the data with label-data.py).
Returns a dictionary in which the key is the frame number and the
value is all of the keypoints and descriptors for that frame.
"""

def cropImageSet(image_set, detector):
    csvfile = open('./gstore-csv/%s.csv' %image_set, 'rb')
    reader = csv.reader(csvfile, delimiter = ',', quotechar = '|')
    Truth = {}
    for row in reader:
        path = './gstore-snippets/%s_snippet/' %image_set
        frame = (5 -len(str(row[0]))) * '0' + str(row[0])
        img = cv2.imread(path + image_set +'_' +frame +'.jpg', 0) #opens correct image in greyscale
        x = int(row[3])
        y = int(row[4])
        x2 = int(row[1])
        y2 = int(row[2])

        crop_img = img[y:y2, x:x2] # NOTE: its img[y: y + h, x: x + w] and *not* img[x: x + w, y: y + h]
        detectorinfo = detector.detectAndCompute(img,None)
        Truth[frame] = siftinfo  
    return

if __name__ == '__main__':
    cropImageSet('cookie')