"""
A script for quick testing of feature detectors with a train image and query image (yay cookies).
"""

import cv2
import numpy as np
from matplotlib import pyplot as plt
import scipy as sp

def visualize(img_t, img_q):
	# Input images
	img1 = cv2.imread('./gstore-snippets/cookie_snippet/%s.jpg' %img_q,0) # queryImage
	img2 = cv2.imread('./OT-res/KP-detect/cookies/%s.jpg' %img_t,0) # trainImage

	# Specify filename for result img
	#print("please type a file name: ")
	#filename = str(raw_input())

	# Initiate detector
	detector = cv2.SIFT()

	# find the keypoints and descriptors with detector of interest
	k1, d1 = detector.detectAndCompute(img1, None)
	k2, d2 = detector.detectAndCompute(img2, None)

	# BFMatcher with default params
	bf = cv2.BFMatcher()
	matches = bf.knnMatch(d1,d2, k=2)

	# Apply ratio test
	good = []
	for m,n in matches:
	    if m.distance < 0.75*n.distance:
	        good.append(m)
	# For displaying the matching lines
	h1, w1 = img1.shape[:2]
	h2, w2 = img2.shape[:2]
	view = sp.zeros((max(h1, h2), w1 + w2, 3), sp.uint8)
	view[:h1, :w1, 0] = img1
	view[:h2, w1:, 0] = img2
	view[:, :, 1] = view[:, :, 0]
	view[:, :, 2] = view[:, :, 0]

	for m in good:
	    # draw the keypoints
	    # print m.queryIdx, m.trainIdx, m.distance
	    color = tuple([sp.random.randint(0, 255) for _ in xrange(3)])
	    cv2.circle(view, (int(k1[m.queryIdx].pt[0]), int(k1[m.queryIdx].pt[1])), 2, [255, 0,0]) 
	    #(int(k2[m.trainIdx].pt[0] + w1), int(k2[m.trainIdx].pt[1])), color)

	# Showing the matches
	cv2.imshow("view", view)
	cv2.waitKey(0)
	#cv2.imwrite('./OT-res/KP-detect/cookies/' + filename,view)

if __name__ == '__main__':
	for frame in range(175, 195):
		visualize('cookie-train', 'cookie_00%d'%frame)