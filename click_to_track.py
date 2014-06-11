import numpy as np
import cv2
import video
import time


class App(object):
    def __init__(self, video_src):
        #Starts video capture (if passing in 0)
        self.cam = video.create_capture(video_src)
        #get the first frame of video
        ret, self.frame = self.cam.read()
        #create a window called camshift
        cv2.namedWindow('camshift')
        #allows to interact with camshift using the mouse
        cv2.setMouseCallback('camshift', self.onmouse)

        #No Selection
        self.selection = None
        #Haven't started dragging
        self.drag_start = None
        #Not tracking anything
        self.tracking_state = 0
        #Not showing black and blue filter that happens when dragging
        self.show_backproj = False

    def onmouse(self, event, x, y, flags, param):
        #sets x and y positions of the mouse
        x, y = np.int16([x, y]) # BUG
        #If we click
        if event == cv2.EVENT_LBUTTONDOWN:
            #start posiiton of the drag
            self.drag_start = (x, y)
            #reset the tracking state
            self.tracking_state = 0
        #If we have clicked to start dragging previously
        if self.drag_start:
            #if we release the mouse to stop dragging
            if flags & cv2.EVENT_FLAG_LBUTTON:
                #define height and width
                h, w = self.frame.shape[:2]
                #one corner of the box (where the click and drag started)
                xo, yo = self.drag_start
                #Leftmost corner of the box
                x0, y0 = np.maximum(0, np.minimum([xo, yo], [x, y]))
                #Rightmost corner of the box
                x1, y1 = np.minimum([w, h], np.maximum([xo, yo], [x, y]))
                #Reset where the box is
                self.selection = None
                #If there is a box
                if x1-x0 > 0 and y1-y0 > 0:
                    #Resets the selection to be the coordinates of the box (defined by two diagonal corners)
                    self.selection = (x0, y0, x1, y1)
            #Right after we create the selection, we need to stop "selecting a box"
            else:
                #We've stoped dragging
                self.drag_start = None
                #If we already have a selection
                if self.selection is not None:
                    #We are tracking something
                    self.tracking_state = 1
    #Show the histogram            
    def show_hist(self):
        #Set the number of bins
        bin_count = self.hist.shape[0]
        #Each bin width is 24
        bin_w = 24
        #Make an empty array that is 256 by the number of bins * 24 by the number of channels
        img = np.zeros((256, bin_count*bin_w, 3), np.uint8)

        #for each bin
        for i in xrange(bin_count):
            #set height of the rectangle
            h = int(self.hist[i])
            #Create a rectangle of each height (the width is already determined)
            cv2.rectangle(img, (i*bin_w+2, 255), ((i+1)*bin_w-2, 255-h), (int(180.0*i/bin_count), 255, 255), -1)
        #Converts color space
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        #Shows the histogram
        cv2.imshow('hist', img)

    #Running the program
    def run(self):
        while True:
            #reads a frame from the camera
            ret, self.frame = self.cam.read()
            #makes a copy of the frame
            vis = self.frame.copy()
            #converts masks
            hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
            #creates a mask (we aren't sure what this mask is used for)
            mask = cv2.inRange(hsv, np.array((0., 60., 32.)), np.array((180., 255., 255.)))

            #If we have something selected (but haven't started tracking yet)
            if self.selection:
                #Set the corners
                x0, y0, x1, y1 = self.selection
                #initialize the tracking window (corner, width, height)
                self.track_window = (x0, y0, x1-x0, y1-y0)
                #Save the hsv of just the range of the region of interest
                hsv_roi = hsv[y0:y1, x0:x1]
                #Mask just this part of the image
                mask_roi = mask[y0:y1, x0:x1]
                #Calculate the histogram for just the region of interest
                hist = cv2.calcHist( [hsv_roi], [0], mask_roi, [16], [0, 180] )
                #Normalizes the histogram
                cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX);
                #Reshape the histogram
                self.hist = hist.reshape(-1)
                #Show the histogram
                self.show_hist()

                #copy the region of interest
                vis_roi = vis[y0:y1, x0:x1]
                #Inverts the region of interest
                cv2.bitwise_not(vis_roi, vis_roi)
                #Wherever the mask = 0, the copy of the frame = 0
                vis[mask == 0] = 0

            #If we're tracking
            if self.tracking_state == 1:
                #Reset the selection
                self.selection = None
                #Calculate the probability of the origninal thing still being in the same place
                prob = cv2.calcBackProject([hsv], [0], self.hist, [0, 180], 1)
                #Bitwise and of prob and mask
                prob &= mask
                #Define termination criteria
                term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )
                #define the tracking box and new window based on probabilities, the tracking window, and the termination criteria
                track_box, self.track_window = cv2.CamShift(prob, self.track_window, term_crit)

                #X is the first coordinate of the tracking box
                xPos = track_box[0][0]

                #If we are showing the back projection
                if self.show_backproj:
                    #everything in probability up until newaxis
                    vis[:] = prob[...,np.newaxis]
                #Try to draw the elipse
                try: cv2.ellipse(vis, track_box, (0, 0, 255), 2)
                #If you can't (track box = none)
                except: print track_box

            #Show the tracking ellipse
            cv2.imshow('camshift', vis)

            #Stop showing things when we press escape
            ch = 0xFF & cv2.waitKey(5)
            if ch == 27:
                break
            #Turn on show back projection by pressing 'b'
            if ch == ord('b'):
                self.show_backproj = not self.show_backproj
        cv2.destroyAllWindows()        

#If we're running this program
if __name__ == '__main__':
    import sys
    #try looking for a file
    try: video_src = sys.argv[1]
    #otherwise use webcam
    except: video_src = 0
    print __doc__
    App(video_src).run()