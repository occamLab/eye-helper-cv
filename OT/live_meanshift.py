import cv2
import numpy as np
from find_keypoints import find_kp
import object_match as om
from compare_kpd import calc_center

"""
Python script that does object tracking in real time with webcam footage. 
Beware the current setup with global variables in the mouse callback functions - 
may or may not create an OOP version of this script for future efforts.
However remember that ultimately the object selection will be on the eye-helper webapp
and probably not on python opencv. - July 28, 2014
"""



### Webcam things

cap = cv2.VideoCapture(0)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('s'): #s for selection mode...
        break

### Selecting things

# Display the frame when 's' key was hit
#cv2.imshow('selection frame', frame)
#cv2.waitKey(0)

### Selecting the item of interest to track (i.e. the training image)

# snippets taken from mousing.py.... beware mouse callbacks and global-ish variables.

# setting state variables
drawing = False # true if mouse is pressed
mode = True # if True, draw rectangle. Press 'm' to toggle to curve
ix,iy = -1,-1
r = None

# mouse callback function
def draw_rectangle(event,x,y,flags,param):
    global ix,iy,drawing,mode,img,r

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix,iy = x,y

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing == True:
            if mode == True:
                img = np.copy(frame)
                cv2.rectangle(img,(ix,iy),(x,y),(0,255,0))

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        if mode == True:
            cv2.rectangle(img,(ix,iy),(x,y),(0,255,0))
            r = [x,y,ix,iy]

# Obtain a copy of the current frame (so there's only one rectangle 
# seen at a time), a window and bind the function to window
img = np.copy(frame)
cv2.namedWindow('image')
cv2.setMouseCallback('image',draw_rectangle)

# while loop for the selection things
while(1):
    cv2.imshow('image',img)
    if cv2.waitKey(20) & 0xFF == 32: #hit spacebar when done 
        break

# Get keypoints of selected area (training image)
t = find_kp(cap.read()[1], 'SIFT', live=True)

while True:
    # Setting up inputs for om.match_object
    previous = calc_center(r)
    current = cap.read()[1] 
    train_img = None #since we're not specifying a training image in a subfolder somewhere else on our computer
    pos = r
    frame = 0

    # Now that we have the training image keypoints, commence object tracking on the video stream!
    center, current = om.match_object(previous, 
                                            current, 
                                            train_img, 
                                            pos, 
                                            frame, 
                                            show = True, 
                                            live = True,
                                            t = t)

    cv2.imshow('OT demo', current)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything's done, release the capture
cap.release()
cv2.destroyAllWindows()