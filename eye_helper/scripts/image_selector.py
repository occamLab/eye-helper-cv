#!/usr/bin/env python

import cv2
import numpy as np

# TODO: testing/debugging in the state machine in eye_helper.py

class ImageSelector():
    def __init__(self):
        t_img = []
        t_img_corners = [] 

        # mouse callback/draw_rectangle things
        self.ix = -1
        self.iy = -1
        self.drawing = False
        self.mode = True
        self.r = None
        self.frozen_img = [] 
        self.current_frame = []

        self.is_drawing = False # is there currently a frozen frame/drawing window?

    # mouse callback function
    def draw_rectangle(self, event, x, y, flags, param):
        self.is_drawing = True

        if event == cv2.EVENT_LBUTTONDOWN: 
            self.drawing = True
            self.ix,self.iy = x,y

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drawing == True:
                if self.mode == True:
                    # copying the frozen image so we have a "fresh canvas" to draw on 
                    # (prevents rectangle ghosting)
                    self.current_frame = np.copy(self.frozen_img)
                    cv2.rectangle(self.current_frame,(self.ix,self.iy),(x,y),(0,255,0))

        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            if self.mode == True:
                cv2.rectangle(self.current_frame,(self.ix,self.iy),(x,y),(0,255,0))
                self.r = [x, y, self.ix, self.iy]  

    def run(self, frame):
        # Obtain a copy of the current frame (so there's only one rectangle 
        # seen at a time), a window and bind the function to window
        self.current_frame = np.copy(frame)
        self.frozen_img = np.copy(frame)
        cv2.namedWindow('selection_window')
        cv2.setMouseCallback('selection_window', self.draw_rectangle)
        # while loop for the selection things
        while(1):
            cv2.imshow('selection_window',self.current_frame)
            if cv2.waitKey(20) & 0xFF == 32: #hit spacebar when done 
                break

if __name__ == '__main__':
    frame = cv2.imread('cookie_00274.jpg')
    im_s = ImageSelector()
    im_s.run(frame)