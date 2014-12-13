#!/usr/bin/env python

import cv2
import numpy as np

class ObjectMatcher():
    def __init__(self):
        self.t_kp = []
        self.t_d = []

        self.q_kp = []
        self.q_d = []

        self.center = None

        self.detector = cv2.SIFT()
        self.matcher = cv2.BFMatcher()
        self.matches = []

    def find_kpd(self, frame, t_img, t_img_corners):
        # TODO: test with hardware

        """
        # get t_img keypoints and descriptors if we don't have them yet
        #   only store the keypoints we care about (i.e. things within corners)
        # get the current frame's keypoints and descriptors
        """

        # Do this once for a particular training image
        if self.t_kp == []:
            x1 = t_img_corners[0]
            y1 = t_img_corners[1]
            x2 = t_img_corners[2]
            y2 = t_img_corners[3]

            # First, find all the keypoints and descriptors for this training image
            t_kp_all, t_d_all = self.detector.detectAndCompute(t_img, None)

            # Look at keypoint coord vals; keep the keypoints within the grocery item selection box
            for index in range(len(t_kp_all)):
                if x1<=t_kp_all[index].pt[0]<= x2 and y1<= t_kp_all[index].pt[1] <=y2:
                    self.t_kp.append(t_kp_all[index])
                    self.t_d.append(t_d_all[index])
            
        # Do this for every frame
        # First, find all the keypoints and descriptors for this query image
        self.q_kp, self.q_d = self.detector.detectAndCompute(frame, None)


    def matching(self):
        # matching happens
        # nearest neighbor tests
        self.matches = []
        try: 
            #match list of object keypoints to query image, not other way around
            all_matches = self.matcher.knnMatch(np.array(self.t_d), self.q_d, k =2)

            #Nearest neighbor test to reduce false matches
            # b/c k=2....
            # m is the nearest match
            # n is the next nearest match
            for m,n in all_matches:
                if m.distance < 0.75*n.distance:
                    # Get coordinate of the match
                    m_x = int(self.q_kp[m.trainIdx].pt[0])
                    m_y = int(self.q_kp[m.trainIdx].pt[1])
                    self.matches.append((m_x, m_y))


        except Exception as inst: #printing the error associated with why the except code chunk ran
            print inst
            print "Likely there are no matches"

    def mean_shift(self):
        # find "center of mass" of the keypoints

        threshold = .25
        if len(self.matches) > 5: # increased to 5 for testing purposes

            #assigns a value to the weighting constant -> based on 
            #experimental results on cropped cookie_00274
            c = 0.0001

            #arbitrarily set diff high to go through loop at least once
            diff = 1000

            while(diff > threshold):

                #sets up lists of weights and weights*position
                x_weights = []
                y_weights = []
                weighted_x = []
                weighted_y = []
                #Creats a list of weighted points, where points near the 
                #hypothesis have a larger weight
                last_guess = self.center
                for kp in self.matches:
                    x_val = np.exp(-c * (kp[0] - last_guess[0])**2)
                    x_weights.append(x_val)
                    weighted_x.append(x_val*kp[0])
                    y_val = np.exp(-c * (kp[1] - last_guess[1])**2)
                    y_weights.append(y_val)
                    weighted_y.append(y_val*kp[1])

                #finds 'center of mass' of the points to determine new center
                x = int(sum(weighted_x)/sum(x_weights))
                y = int(sum(weighted_y)/sum(y_weights))

                #update hypothesis
                self.center = (x,y)

                #difference between the current and last guess
                diff = np.sqrt((last_guess[0] - x)**2 + (last_guess[1] - y)**2)
            
    def draw_circles(self, radius, frame):
        """Takes in:
            radius -> estimated radius of object
        """
        if self.matches != None:
            for k in self.matches:
                cv2.circle(frame, (k[0], k[1]), 1, [255, 0, 0], 2)
        if self.center != None:
            cv2.circle(frame, self.center, 6, [0,0,255], 6)
        # cv2.imshow('frame with center', frame)
        # cv2.waitKey(0)

    def run(self, frame, t_img, t_corners):
        """
        execute find_kpd(), matching(), and mean_shift()

        return:
            center
        """

        self.find_kpd(frame, t_img, t_corners)
        self.matching()
        self.mean_shift()
        self.draw_circles(5, frame)

        # TODO: manipulating the frame... make a new window?
        # visualizes moving center and displays keypoints if show==True for the frame 
        #(so, this happens once per call of the function)
        
        return self.center

if __name__ == '__main__':
    t_img = cv2.imread('cookie_00274.jpg')
    t_img_corners = [126, 268, 786, 652] 
    ob = ObjectMatcher(t_img, t_img_corners)
    
    q_img = cv2.imread('cookie_00281.jpg')
    ob.run_match(q_img)
