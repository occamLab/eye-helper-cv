import cv2
import numpy as np
from matplotlib import pyplot as plt
import scipy as sp

img1 = cv2.imread('cookie-query.jpg',0) # queryImage
img2 = cv2.imread('cookie_00270_train.jpg',0) # trainImage

# Initiate SIFT detector
sift = cv2.SIFT()

# find the keypoints and descriptors with SIFT
k1, d1 = sift.detectAndCompute(img1,None)
k2, d2 = sift.detectAndCompute(img2,None)

# BFMatcher with default params
bf = cv2.BFMatcher()
matches = bf.knnMatch(d1,d2, k=2)

# Apply ratio test
good = []
for m,n in matches:
    if m.distance < 0.75*n.distance:
        good.append(m)



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
    cv2.line(view, (int(k1[m.queryIdx].pt[0]), int(k1[m.queryIdx].pt[1])) , (int(k2[m.trainIdx].pt[0] + w1), int(k2[m.trainIdx].pt[1])), color)


cv2.imshow("view", view)
cv2.waitKey(0)
cv2.imwrite('cookie_00270_res.jpg',view)

# cv2.drawMatchesKnn expects list of lists as matches.


# img = cv2.imread('cookie_00270.jpg')
# gray= cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

# sift = cv2.SIFT()
# kp, des = sift.detectAndCompute(gray,None)
# print des

# img=cv2.drawKeypoints(gray,kp)

# cv2.imwrite('sift_keypoints.jpg',img)