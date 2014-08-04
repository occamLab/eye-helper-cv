# import cv2
# import numpy as np
# from matplotlib import pyplot as plt

# img1 = cv2.imread('posterLeft.jpg',0)  #queryimage # left image
# img2 = cv2.imread('posterRight.jpg',0) #trainimage # right image

# sift = cv2.SIFT()

# # find the keypoints and descriptors with SIFT
# kp1, des1 = sift.detectAndCompute(img1,None)
# kp2, des2 = sift.detectAndCompute(img2,None)

# # FLANN parameters
# FLANN_INDEX_KDTREE = 0
# index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
# search_params = dict(checks=50)

# flann = cv2.FlannBasedMatcher(index_params,search_params)
# matches = flann.knnMatch(des1,des2,k=2)

# good = []
# pts1 = []
# pts2 = []

# # ratio test as per Lowe's paper
# for i,(m,n) in enumerate(matches):
#     if m.distance < 0.8*n.distance:
#         good.append(m)
#         pts2.append(kp2[m.trainIdx].pt)
#         pts1.append(kp1[m.queryIdx].pt)
# print len(good)

# pts1 = np.int32(pts1)
# pts2 = np.int32(pts2)

# pts1 = np.array([[float(pt[0]), float(pt[1])] for pt in pts1])
# pts2 = np.array([[float(pt[0]), float(pt[1])] for pt in pts2])


# F, mask = cv2.findFundamentalMat(pts1,pts2,cv2.FM_LMEDS)

# # We select only inlier points
# pts1 = pts1[mask.ravel()==1]
# pts2 = pts2[mask.ravel()==1]



# print F 


import cv2
import numpy as np 
from matplotlib import pyplot as plt
import scipy as sp 

def cameraCalib(camera):
    """Walks through getting callibration matrix for a camera"""

    cap = cv2.VideoCapture(camera)
    test_imgs = []
    good = 0
    while good <=15:
        while(True):
            ret, frame = cap.read()
            cv2.imshow('test', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): #s for selection mode...
                break
        
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (8,6),None)
        sift = cv2.SIFT()
        kp, d = sift.detectAndCompute(gray, mask = None)
        for k in kp:
            cv2.circle(gray, (int(k.pt[0]), int(k.pt[1])), 2, [0, 0, 255], 2)
        cv2.imshow('frame', gray)
        cv2.waitKey(0)
        cv2.destroyWindow('frame')
        print ret
        print corners
        if ret:
            cv2.imwrite('img_%d.jpg' %good, frame)
            good +=1


if __name__ == '__main__':
    cameraCalib(1)


# import numpy as np
# import cv2
# import glob

# # termination criteria
# criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
# objp = np.zeros((9*7,3), np.float32)
# objp[:,:2] = np.mgrid[0:9,0:7].T.reshape(-1,2)

# # Arrays to store object points and image points from all the images.
# objpoints = [] # 3d point in real world space
# imgpoints = [] # 2d points in image plane.

# images = glob.glob('*.jpg')

# for fname in images:
#     img = cv2.imread(fname)
#     gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

#     # Find the chess board corners
#     ret, corners = cv2.findChessboardCorners(gray, (6,7),None)
#     print corners
#     print fname
#     # If found, add object points, image points (after refining them)
#     if ret == True:
#         objpoints.append(objp)

#         corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
#         imgpoints.append(corners2)

#         # Draw and display the corners
#         img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)
#         cv2.imshow('img',img)
#         cv2.waitKey(500)

# cv2.destroyAllWindows()