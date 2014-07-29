import cv2
import numpy as np 
import scipy as sp 

"""
Object matching shenanigans with meanshift. 

--Emily and Lindsey, July 25, 2014
"""

def match_object(previous, current, train_img, pos, frame=0, show = False, live = False, t = ()):
    """
    Takes in:
        previous -> center of the previous image
        current ->  image being analyzed (if not live: the img path)
        train_img -> training image for keypoint matching
        pos -> corners (left top, right bottom) of object in training img
        show -> determines if visualization is displayed
    Output:
        new_center -> new center determined by meanshift
                      if there are no matches, original center is returned instead

    """

    print pos

    # corners of selected item in training image, accounts for the flipping
    x1 = pos[2]
    y1 = pos[3]
    x2 = pos[0]
    y2 = pos[1]

    #I choose YOU! SIFT-emon
    detector = cv2.SIFT()

    if not live:
        #Take in training image with coordinates of tracked object
        t_img = cv2.imread(train_img)

        #query image
        q_img = cv2.imread(current)

        #Keeps only keypoints within the grocery item selection
        t_k, t_d = detector.detectAndCompute(t_img, None)   #training image

        train_d = []
        train_k = []

        for index in range(len(t_k)):
            if x1<=t_k[index].pt[0]<= x2 and y1<= t_k[index].pt[1] <=y2:
                train_k.append(t_k[index])
                train_d.append(t_d[index])
        t_k = train_k
        t_d = train_d


    else:
        #to account for how the webcam version of this deals with inputs differently
        # t_img = train_img #should be cap.read()[1] 
        q_img = current
        t_k = t[0] 
        t_d = t[1]

        train_d = []
        train_k = []

        for index in range(len(t_k)):
            if x1<=t_k[index][0]<= x2 and y1<= t_k[index][1] <=y2:
                train_k.append(t_k[index])
                train_d.append(t_d[index])
        t_k = train_k
        t_d = train_d

    #finds all the keypoints in the query image
    q_k, q_d = detector.detectAndCompute(q_img, None)   #query image

    try:
        matcher = cv2.BFMatcher() #(normType = cv2.NORM_HAMMING)

        #match list of object keypoints to query image, not other way around
        matches = matcher.knnMatch(np.array(t_d), q_d, k =2)

        #Nearest neighbor test to reduce false matches
        good_matches = []
        for m,n in matches:
            if m.distance < 0.75*n.distance:
                # Get coordinate of the match
                m_x = int(q_k[m.trainIdx].pt[0])
                m_y = int(q_k[m.trainIdx].pt[1])
                good_matches.append((m_x, m_y))

        # for the sake of visualizing the original center. should not affect OT functionality. 
        # img = np.copy(q_img)
        # cv2.circle(img, (previous[0], previous[1]), 3, [0, 0, 255], 3)
        # cv2.imshow('Frame %d: original center' % (frame), img)
        # cv2.imwrite('../OT_res/meanshift_presentation/cookie_f%d_center_original.jpg' % (frame), img)
        # cv2.waitKey(0)

        new_center, img_radius = mean_shift(hypothesis = (previous), 
                                            keypoints = good_matches, 
                                            threshold = 10, 
                                            current = q_img,
                                            show = show, 
                                            frame = frame, 
                                            live = True, 
                                            show_iterations = False)
        return new_center

    except Exception as inst: #printing the error associated with why the except code chunk ran
        print inst
        print "Likely there are no matches"
        return previous

def mean_shift(hypothesis, keypoints, threshold, frame, current = None, show = False, live = False, show_iterations = False):
    """
    Inputs:
        hypothesis -> Previous center point as a starting hypothesis
        keypoints -> List of keypoint (x,y) coordinates
        Threshold -> maximum acceptable difference in center between iterations (eg 10 pixels, 5 pixels)
        current -> np array representing the image (for visualization)
        show -> determines whether visualization is shown

    Returns:
        New center of keypoints
        Radius
        If show is true -> displays the center, keypoints and a circle around the object
    """

    n=0
    if len(keypoints) > 1:

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
            last_guess = hypothesis
            for kp in keypoints:
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
            hypothesis = (x,y)

            #difference between the current and last guess
            diff = np.sqrt((last_guess[0] - x)**2 + (last_guess[1] - y)**2)

            # Finding the radius:
            norm_weights = [np.linalg.norm([x_weights[i], y_weights[i]]) for i in range(len(x_weights))]
            avg_weight = sum(norm_weights)/len(norm_weights)
            std_weight = np.std(norm_weights)

            # Threshold based on standard deviations (to account for different kp density scenarios)
            threshold = avg_weight #- .25*std_weight
            inliers = []

            # Radius corresponds to the farthest-away keypoints are in the threshold from center of mass (x,y)
            for index in range(len(norm_weights)):
                if norm_weights[index] > threshold:
                    coords = [keypoints[index][0] - x, keypoints[index][1] - y] 
                    inliers.append(np.linalg.norm(coords))
            radius = int(max(inliers))

            #visualizes moving center and displays keypoints for every meanshift iteration if show_iterations==True
            if show_iterations:
                img = np.copy(current)   #Needs to be np array (already opened by cv2)
                for k in keypoints:
                    cv2.circle(img, k, 2, [255, 0, 0], 2)
                cv2.circle(img, hypothesis, 3, [0, 0, 255], 3)
                cv2.circle(img, hypothesis, radius, [100,255,0], 2)
                cv2.imshow('Frame %d: Current hypothesis, meanshift guess %d' % (frame, n), img)
                # cv2.imwrite('../OT_res/meanshift_presentation/cookie_f%d_guess%d.jpg' % (frame, n), img)
                cv2.waitKey(0)
                n+=1
        
        #visualizes moving center and displays keypoints if show==True for the frame 
        #(so, this happens once per call of the function)
        if show:
            img = np.copy(current)   #Needs to be np array (already opened by cv2)
            for k in keypoints:
                cv2.circle(img, k, 2, [255, 0, 0], 2)
            cv2.circle(img, hypothesis, 3, [0, 0, 255], 3)
            cv2.circle(img, hypothesis, radius, [100,255,0], 2)
            cv2.imshow('Frame %d: Current hypothesis' % (frame), img)
            # cv2.imwrite('../OT_res/meanshift_presentation/cookie_f%d_guess%d.jpg' % (frame, n), img)
            cv2.waitKey(0)

        return hypothesis, radius

    # elif len(keypoints) == 1: # That moment when there's only one good match and the stdev of a single element set is zero...

    #     hypothesis = (keypoints[0][0], keypoints[0][1])
    #     radius = 10

    #     #visualizes moving center and displays keypoints
    #     if show:
    #         img = np.copy(current)   #Needs to be np array (alread opened by cv2)
    #         cv2.circle(img, hypothesis, 3, [0, 0, 255], 3)
    #         cv2.circle(img, hypothesis, radius, [100,255,0], 2)
    #         # cv2.imwrite('./OT-res/meanshift/cookie/cookie_00%d.jpg' % frame, img)
    #         cv2.imshow('Frame %d: Current hypothesis' % frame, img)
    #         cv2.waitKey(0)

    #     return hypothesis, radius
 
if __name__ == '__main__':
    # initial values for prototyping w/ the cookies. :P
    old_center = [1000, 170]
    center = old_center
    pos = [744,514,606,392]

    for frame in range(180, 182): 
        cv2.destroyAllWindows()
        print "Frame number: %d" % frame
        center = match_object(previous = center, 
                              current = '../gstore_snippets/cookie_snippet/cookie_00%d.jpg' % frame, 
                              train_img = '../gstore_snippets/cookie_snippet/cookie_00177.jpg',
                              pos = pos,
                              show = True,
                              frame = frame)
        old_center = center
  