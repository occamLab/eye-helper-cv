import cv2
import numpy as np
from matplotlib import pyplot as plt
import glob
import numpy.linalg 


def in_front_of_both_cameras(first_points, second_points, rot, trans):
    # check if the point correspondences are in front of both images
    rot_inv = rot
    for first, second in zip(first_points, second_points):
        first_z = np.dot(rot[0, :] - second[0]*rot[2, :], trans) / np.dot(rot[0, :] - second[0]*rot[2, :], second)
        first_3d_point = np.array([first[0] * first_z, second[0] * first_z, first_z])
        second_3d_point = np.dot(rot.T, first_3d_point) - np.dot(rot.T, trans)
 
        if first_3d_point[2] < 0 or second_3d_point[2] < 0:
            return False
 
    return True


def extract_r_t_f(im1, im2, mtx, dst):

    """
    This function will find the essential matrix from an estimation of the 
    fundamental matrix and from that we shall extract rotation and translation
    vectors.  This information may later be used to help determine the depth of 
    various objects in the image. In the course of finding the fundamental matrix
    we are also getting a second layer of false match elimination.

    Inputs:
        im1 -> first image file path
        im2 -> second image file path

    Returns:
        pts1, pts2 -> list of points that based on their position match well
                      enough with other image points to make the fundamental matrix
        T -> translation matrix
        R -> rotation matrix
        F -> fundamental matrix

    Reference: 
        A large amount of the code in this function (anything from essential matrix on)
        comes from code developed here: https://gist.github.com/jensenb/8668000
    """
    # K = [[ 598.28339238    0.          338.53221923]
    #    [   0.          595.80675436  230.06429972]
    #    [   0.            0.            1.        ]]

    #d = [[ 0.10643171 -0.55556305 -0.00786038  0.00290519  0.98123148]]

    #d the distortion matrix with three extra zeros on the end...we're not yet sure why the zeros are there
    # d = np.array([ 0.10643171, -0.55556305, -0.00786038,  0.00290519,  0.98123148, 0.0, 0.0, 0.0]).reshape(1,8)
    # #K the camera matrix (contains intrinsic camrea parameters)
    # K = np.array([598.28339238, 0.0, 338.53221923, 0.0, 595.80675436,  230.06429972, 0.0, 0.0, 1.0]).reshape(3,3)
    d = dst 
    K = mtx

    #K_inv is the inverse of K
    K_inv = np.linalg.inv(K)
    
    img1 = cv2.imread(im1,0)  #queryimage # left image
    img2 = cv2.imread(im2,0) #trainimage # right image

    sift = cv2.SIFT()

    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(img1,None)
    kp2, des2 = sift.detectAndCompute(img2,None)

    # FLANN parameters
    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks=50)

    flann = cv2.FlannBasedMatcher(index_params,search_params)
    matches = flann.knnMatch(des1,des2,k=2)

    good = []
    pts1 = []
    pts2 = []

    # ratio test as per Lowe's paper
    for i,(m,n) in enumerate(matches):
        if m.distance < 0.8*n.distance:
            good.append(m)
            pts2.append(kp2[m.trainIdx].pt)
            pts1.append(kp1[m.queryIdx].pt)

    pts1 = np.int32(pts1)
    pts2 = np.int32(pts2)

    pts1 = np.array([[float(pt[0]), float(pt[1])] for pt in pts1])
    pts2 = np.array([[float(pt[0]), float(pt[1])] for pt in pts2])


    F, mask = cv2.findFundamentalMat(pts1,pts2,cv2.FM_LMEDS)

    # We select only inlier points
    pts1 = pts1[mask.ravel()==1]
    pts1 = [np.append(pt, 0.0) for pt in pts1]
    pts2 = pts2[mask.ravel()==1]
    pts2 = [np.append(pt, 0.0) for pt in pts2]

    #Decompsing the essential matrix from the fundamental matrix based on MATH!
    E = K.T.dot(F).dot(K)

    #Singular value decomposition
    #U and V are the unitary matrices -> not clear on what these are
    #S are the singular values
    U, S, V = np.linalg.svd(E)
    W = np.array([0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]).reshape(3, 3)

    R = U.dot(W).dot(V)
    T = U[:, 2]

    if not in_front_of_both_cameras(pts1, pts2, R, T):
        # Second choice: R = U * W * Vt, T = -u_3
        T = - U[:, 2]
        if not in_front_of_both_cameras(pts1, pts2, R, T):
            # Third choice: R = U * Wt * Vt, T = u_3
            R = U.dot(W.T).dot(V)
            T = U[:, 2]
     
            if not in_front_of_both_cameras(pts1, pts2, R, T):
                # Fourth choice: R = U * Wt * Vt, T = -u_3
                T = - U[:, 2]
    
    return R, T, F, pts1, pts2

def rectify(K, d, R, T, F, img1_path, img2_path, pts1, pts2):
    """
    Performs the image rectification that needs to happen before calculuating disparity. 

    Inputs:
    K -> intrinsic camera parameters matrix
    d -> camera distortion parameters matrix
    R -> rotation from one camera to another
    T -> translation from one camera to another
    F -> fundamental matrix
    img1_path -> filepath. generally speaking, the left-hand-side image 
    img2_path -> filepath. generally speaking, the right-hand-side image
    pts1 -> the matched point locations in image 1
    pts2 -> the matched point location in image2

    """

    img_1 = cv2.imread(img1_path,0)
    img_2 = cv2.imread(img2_path,0)
    size = img_1.shape
    print size

    cv2.stereoRectifyUncalibrated(points1=pts1, points2=pts2, F=F, imgSize=size)

    ##### image rect. slide deck
    # #step 1: elements of Rrect (following guide in image rectification slide deck)
    # e1 = T / np.linalg.norm(T)
    # e2 = ( 1 / np.sqrt( T[0]**2 + T[1]**2 ) ) * np.array([-T[1], T[0], 0]).T 
    # e3 = np.cross(e1, e2)
    # R_rect = np.array([e1.T, e2.T, e3.T])

    # # step 2
    # R_L = R_rect
    # R_R = np.dot(R, R_rect)

    # step 3-4... to be done soon!

    #prototype webcam has focal length of 2.0
    #google glass camera has focal length of 3.0, according to interwebs.
    ######


    # cv2.imshow('img_rect1', img_rect1)
    # cv2.imshow('img_rect2', img_rect2)
    # cv2.waitKey(0)

    #result naming convension:
    #rect_0_1 and rect_1_0, etc. (they come in pairs)

    return img_rect1, img_rect2


def grab_calib_pics(camera):
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
        if ret:
            cv2.imwrite('img_%d.jpg' %good, frame)
            good +=1

def calibrate_from_chessboard():
    """
    Assumes that there are at least ten jpg images of the chessboard callibration rig 
    from different views and that these are the only jpg images in this folder

    returns:
    Camera matrix (intrinsic camera parameters)
    distortion matrix (of camera)
    """

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((8*6,3), np.float32)
    objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    images = glob.glob('*.jpg')

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (8,6),None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            imgpoints.append(corners)


    # cv2.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

    return mtx, dist, rvecs, tvecs




if __name__ == '__main__':
    mtx, dst, rvecs, tvecs = calibrate_from_chessboard()
    # np.append(dst, [0.0, 0.0, 0.0]) 
    R,T, F, pts1, pts2 = extract_r_t_f('img_0.jpg', 'img_1.jpg', mtx, dst)
    center = np.array([0.0, 0.0, 0.0]).reshape(1,3)
    new_center = (center+T).dot(R)
    print 'R: ', R 
    print 'T: ', T
    # print 'first center: ', center
    # print 'new center: ', new_center

    pts1 = np.array(pts1)
    pts2 = np.array(pts2)

    rectify(mtx, dst, R, T, F, 'img_0.jpg', 'img_1.jpg', pts1, pts2)
