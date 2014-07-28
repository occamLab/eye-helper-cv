import cv2
import numpy as np

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
    global ix,iy,drawing,mode,img, r

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
            r = [(ix,iy),(x,y)]

# Create a black image, a window and bind the function to window
img = np.copy(frame)
cv2.namedWindow('image')
cv2.setMouseCallback('image',draw_rectangle)

# while loop for the selection things
while(1):
    cv2.imshow('image',img)
    if cv2.waitKey(20) & 0xFF == 32: #hit spacebar when done 
        break

# get keypoints of selected area (training image)

# now that we have the training image keypoints, commence object tracking on the video stream!

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()