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
            good_matches.append([m_x, m_y])

    #Creates a mask with a probability of 1 where keypoint are and 0 where they aren't
    prob_image = np.zeros((q_img.shape[0], q_img.shape[1]))
    for match in good_matches:
        prob_image[match[1]][match[0]] = 255
    print good_matches
    cv2.imshow('mask', prob_image)
    cv2.waitKey(0)

    #Using opencv's build in backprojection functions


    #Attempting CAMshift black magic -> hopefully it will work even though this isn't a histogram
    term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
    r,c,w,h = 129,279,654,372
    did_it_do_it, new_window = cv2.CamShift(prob_image, (c,r,w,h), term_crit)

    x,y,w,h = new_window
    if did_it_do_it == True:
        cv2.rectangle(current, (x,y), (x+w, y+h), (255,0, 0), 2)
        cv2.imshow('Feast Your Eyes!', current)
        cv2.waitKey(0)
    else:
        print "I am sorry I have failed you"


def mean_shift(hypothesis, keypoints, threshold):
    """
    Inputs:
        Previous center point as a starting hypothesis
        List of keypoint (x,y) coordinates

    Returns:
        New center of keypoints
        (At some point hopefully also a radius)
    """

    #assigns a value to the weighting constant -> based on 
    #experimental results
    c = 1

    #arbitrarily set diff high to go through loop at least once
    diff = 100

    while(diff > threshold)
        #sets up lists of weights and weights*position
        x_weights = []
        y_weights = []
        weighted_x = []
        weighted_y = []
        #Creats a list of weighted points, where points near the 
        #hypothesis have a larger weight
        for index in range(len(keypoints)):
            x_val = np.exp(-c * (keypoints[0][index] - hypothesis[0])**2)
            x_weights.append(x_val)
            weighted_x.append(x_val*keypoints[0])
            y_val = np.exp(-c * (keypoints[1][index] - hypothesis[1])**2)
            y_weights.append(y_val)
            weighted_y.append(y_val*keypoints[1])

        #finds 'center of mass' of the points to determine new center
        x = sum(weighted_x)/sum(x_weights)
        y = sum(weighted_y)/sum(y_weights)

        #update hypothesis
        hypothesis = (x,y)

        diff = np.sqrt((keypoints[0] - x)**2 + (keypoints[1] - y)**2)
    return hypothesis

def crop_and_display(previous, current, train_img):
    """
    Takes in:
        Corners of the previous image as a list [x, y, x2, y2]
        current image with path
        training image for keypoint matching
    Outputs:
        displays cropped image with matching keypoints


    """
    img = cv2.imread(current, 0) #opens correct image in greyscale
    t_img = cv2.imread(train_img, 0)
    #takes in the coreners of where the object was before and adds 
    #a tolerance 
    x = previous[0] -75
    y = previous[1] -75
    x2 = previous[2] +75
    y2 = previous[3] +75

    #crops image to reduce the neccessary search area
    crop_img = img[y:y2, x:x2] # NOTE: its img[y: y + h, x: x + w] and *not* img[x: x + w, y: y + h]

    detector = cv2.ORB()

    t_k, t_d = detector.detectAndCompute(t_img, None)         #training image
    q_k, q_d = detector.detectAndCompute(crop_img, None)      #query image

    matcher = cv2.BFMatcher(normType = cv2.NORM_HAMMING)
    matches = matcher.knnMatch(q_d, t_d, k =2)

    good_matches = []
    for m,n in matches:
        if m.distance < 0.75*n.distance:
            # Get coordinate of the match
            m_x = int(q_k[m.queryIdx].pt[0])
            m_y = int(q_k[m.queryIdx].pt[1])
            good_matches.append(m)
    print good_matches
    
    h1, w1 = crop_img.shape[:2]
    h2, w2 = t_img.shape[:2]
    view = sp.zeros((max(h1, h2), w1 + w2, 3), sp.uint8)
    view[:h1, :w1, 0] = crop_img
    view[:h2, w1:, 0] = t_img
    view[:, :, 1] = view[:, :, 0]
    view[:, :, 2] = view[:, :, 0]

    for m in good_matches:
        # draw the keypoints
        cv2.circle(view, (int(q_k[m.queryIdx].pt[0]), int(q_k[m.queryIdx].pt[1])), 2, [0, 0, 255], 2)
    # Showing the matches
    cv2.imshow("view", view)
    cv2.waitKey(0)

if __name__ == '__main__':
    match_object(previous =[126,268,786,652], 
                 current = './gstore-snippets/cookie_snippet/cookie_00274.jpg', 
                 train_img = './OT-res/KP-detect/cookie/cookie-train.jpg')