import cv2
import numpy as np 
import scipy as sp 


def match_object(previous, current, train_img):
    """

    Takes in:
        center of the previous image
        current image with path
        training image for keypoint matching
    Outputs:
        ??

    """
    #opens training and query images

    t_img = cv2.imread(train_img)
    img = cv2.imread(current)

    x = previous[0] -75
    y = previous[1] -75
    x2 = previous[2] +75
    y2 = previous[3] +75

    #crops image to reduce the neccessary search area
    q_img = img[y:y2, x:x2] # NOTE: its img[y: y + h, x: x + w] and *not* img[x: x + w, y: y + h]

    #I choose YOU! ORB-achu
    detector = cv2.ORB()

    t_k, t_d = detector.detectAndCompute(t_img, None)         #training image
    q_k, q_d = detector.detectAndCompute(q_img, None)      #query image

    matcher = cv2.BFMatcher(normType = cv2.NORM_HAMMING)
    matches = matcher.knnMatch(q_d, t_d, k =2)

    #Keeping only matches that pass a 2nd nearest neighbor test
    good_matches = []
    for m,n in matches:
        if m.distance < 0.75*n.distance:
            # Get coordinate of the match
            m_x = int(q_k[m.queryIdx].pt[0])
            m_y = int(q_k[m.queryIdx].pt[1])
            good_matches.append((m_x, m_y))
    return good_matches, q_img


def mean_shift(hypothesis, keypoints, threshold, current = None, show = False):
    """
    Inputs:
        Previous center point as a starting hypothesis
        List of keypoint (x,y) coordinates

    Returns:
        New center of keypoints
        (At some point hopefully also a radius)
    """

    #assigns a value to the weighting constant -> based on 
    #experimental results on cropped cookie_00274
    c = 0.00001

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
        last_guess = hypothesis
        for kp in keypoints:
            print kp
            x_val = np.exp(-c * (kp[0] - last_guess[0])**2)
            print x_val
            x_weights.append(x_val)
            weighted_x.append(x_val*kp[0])
            y_val = np.exp(-c * (kp[1] - last_guess[1])**2)
            y_weights.append(y_val)
            weighted_y.append(y_val*kp[1])

        #finds 'center of mass' of the points to determine new center
        x = int(sum(weighted_x)/sum(x_weights))
        print x
        y = int(sum(weighted_y)/sum(y_weights))
        print y

        #update hypothesis
        hypothesis = (x,y)

        diff = np.sqrt((last_guess[0] - x)**2 + (last_guess[1] - y)**2)

        #visualizes moving center and displays keypoints
        if show:
            img = current   #Remember to switch back to imread once done debugging
            for k in keypoints:
                cv2.circle(img, k, 2, [255, 0, 0], 2)
            cv2.circle(img, hypothesis, 3, [0, 0, 255], 3)
            cv2.imshow('Current hypothesis', img)
            cv2.waitKey(0)
    
    return hypothesis



if __name__ == '__main__':
    keypoints, current = match_object(previous =[126,268,786,652], 
                                      current = './gstore-snippets/cookie_snippet/cookie_00274.jpg', 
                                      train_img = './OT-res/KP-detect/cookie/cookie-train.jpg')
    mean_shift(hypothesis = (760, 470), 
               keypoints = keypoints, 
               threshold = 10, 
               current = current,
               show = True)