import cv2
import numpy as np 
import csv

def compare_dm(videoname, trainimg):
    """
    July 8, 2014 (happy birthday Lindsey!)
    Emily and Lindsey's function for for comparing OpenCV methods for feature 
    detection and matching. 

    Function takes in the videoname of interest (should already be in the 
    gstore-snippets folder) and trainimg (for now, this is the explicit
    path to the training image in the repo).
    The videoname is used to obtain the frames of interest and get the 
    appropriate ground truth csv from the gstore-csv folder.

    Function returns a dictionary of the form...
    key: keypoint detection method 
    value: success rate (as a percentage)

    """    

    # Video snippet path
    path = './gstore-snippets/%s_snippet/' %videoname

    # Methods of interest to loop through
    d_methods = ['SIFT', 'ORB', 'BRISK', 'SURF']

    # For calculating the success ratio...

    # Dictionary in which the key is the method and value is the success percentage
    # "success" means the match agrees with the ground truth
    successes = {}

    # Loop through the methods, store in a dictionary somehow
    for method in d_methods:
        correctmatches = 0
        totalmatches = 0

        # Opening the ground truth csv
        csvfile = open('./gstore-csv/%s-query.csv' %videoname, 'rb')
        reader = csv.reader(csvfile, delimiter = ',', quotechar = '|')
        print method

        # Initiate the detector and descriptor
        # detector and descriptor have the same method name
        detector = cv2.FeatureDetector_create(method)
        descriptor = cv2.DescriptorExtractor_create(method) 

        # Open the training image 
        t = cv2.imread(trainimg,0)

        # Get the training image's descriptors and keypoints
        t_k = detector.detect(t)
        t_k, t_d = descriptor.compute(t, t_k) 

        # Loop through rows in the csv (i.e. the video frames)
        for row in reader:
            # Current frame of interest
            frame = (5 -len(str(row[0]))) * '0' + str(row[0]) #i.e. frame number
            imname = "%s%s_%s.jpg"%(path, videoname, frame)
            
            # top left corner (x, y) of ground truth box
            x = int(row[3]) 
            y = int(row[4])

            # bottom right corner (x2, y2) of ground truth box
            x2 = int(row[1]) 
            y2 = int(row[2])

            # Open the current frame
            im = cv2.imread(imname,0)

            # Using the current method, get the keypoints for each frame
            im_k = detector.detect(im)
            im_k, im_d = descriptor.compute(im, im_k) 

            # BFMatcher with default params
            bf = cv2.BFMatcher()
            matches = bf.knnMatch(t_d, im_d, k=2)

            # Apply ratio test
            for m,n in matches:
                if m.distance < 0.75*n.distance:
                    # Get coordinate of the match
                    m_x = int(im_k[m.queryIdx].pt[0])
                    m_y = int(im_k[m.queryIdx].pt[1])
                    
                    # Increment totalmatches
                    totalmatches += 1

                    # Increment correctmatches if this match agrees with the ground truth
                    if x <= m_x <= x2 and y <= m_y <= y2:
                        correctmatches += 1

        # Compute success ratios for all the rows for this particular method
        #successes[method] = float(correctmatches)/float(totalmatches) * 100
        successes[method] = [correctmatches, totalmatches, float(correctmatches)/float(totalmatches)*100]

    # Return dictionary of method: success ratio
    return successes


if __name__ == '__main__':
    print compare_dm('cookie', './OT-res/KP-detect/cookies/cookie-train.bmp')

