import cv2
import numpy as np 
import csv
from matplotlib import pyplot as plt
import scipy as sp
import time
import pickle

def kpvisualize(img_t, img_q, box, good_matches, t_k, q_k):
    """ 
    Displays the ground truth box and matches on the query image

        inputs: training image, query image, box corners, list of good matches,
                training keypoints, query keypoints

        outputs: image with points and box plotted
    """
    # Input images
    img1 = cv2.imread(img_q,0) # queryImage
    img2 = cv2.imread(img_t,0) # trainImage

    # For displaying the matching circles
    h1, w1 = img1.shape[:2]
    h2, w2 = img2.shape[:2]
    view = sp.zeros((max(h1, h2), w1 + w2, 3), sp.uint8)
    view[:h1, :w1, 0] = img1
    view[:h2, w1:, 0] = img2
    view[:, :, 1] = view[:, :, 0]
    view[:, :, 2] = view[:, :, 0]

    for m in good_matches:
        # draw the keypoints
        cv2.circle(view, (int(q_k[m.queryIdx].pt[0]), int(q_k[m.queryIdx].pt[1])), 2, [0, 0, 255], 2)
    
    #box is a list of corners forming a box
    cv2.rectangle(view, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), [255, 0,0]) 

    # Showing the matches
    cv2.imshow("view", view)
    cv2.waitKey(0)


def compare_dm(videoname, trainimg, gt_csv, visualize = False):
    """
    July 8, 2014 (happy birthday Lindsey!)
    Emily and Lindsey's function for for comparing OpenCV methods for feature 
    detection and matching. 

    Inputs:
    videoname -> of interest (should already be in the gstore-snippets folder) 
        The videoname is used to obtain the frames of interest and get the 
        appropriate ground truth csv from the gstore-csv folder.
    trainimg -> (for now, this is the explicit path to the training image in the repo)
    gt_csv -> csv with coordinates of ground truth box
    visualize -> if true plots matched keypoints and ground truth box
    
    Function returns a dictionary of the form...
    key: keypoint detection method 
    value: success rate (as a percentage)

    Things to test with for the performance of this function:
    Measuring time per step
    Iterating through matching techniques
    Subtract the background keypoints before matching
    Angle/POV changes
    Loop for different matching techniques

    """    

    # Video snippet path
    path = './gstore-snippets/%s_snippet/' %videoname

    # Methods of interest to loop through
    d_methods = ['SIFT', 'ORB', 'BRISK']# 'SURF']

    # For calculating the success ratio...

    # Dictionary in which the key is the method and value is the success percentage
    # "success" means the match agrees with the ground truth
    successes = {}

    # Loop through the methods, store in a dictionary somehow
    for method in d_methods:

        print "starting %s" % method
        #Initializing things for running the loop on this method
        method_start_time = time.time()
        frametimes = {'open_time':[], 
                      'kp_detect_time':[],
                      'match_time':[],
                      'vis_time':[],
                      'start_to_end_time':[]}
        correctmatches = 0
        totalmatches = 0
        good_matches = []
        frame_total_times = []
        f_successes = []

        # Opening the ground truth csv
        csvfile = open('./gstore-csv/%s' % gt_csv, 'rb')            
        reader = csv.reader(csvfile, delimiter = ',', quotechar = '|')

        # Initiate the detector and descriptor
        # detector and descriptor have the same method name
        detector = cv2.FeatureDetector_create(method)
        descriptor = cv2.DescriptorExtractor_create(method) 

        # Open the training image 
        t = cv2.imread(trainimg,0)
        print 'trainimg is %s' % trainimg

        # Get the training image's descriptors and keypoints
        t_k = detector.detect(t)
        t_k, t_d = descriptor.compute(t, t_k) 

        # Loop through rows in the csv (i.e. the video frames)
        for row in reader:
            # So we can get %accuracy per frame
            f_totalmatches = 0
            f_correctmatches = 0            

            # print row
            # Time: Starting this frame
            frame_start_time = time.time()

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

            # Time: How long it took to open the current frame
            frame_open_time = time.time()
            frametimes['open_time'].append(frame_open_time - frame_start_time)

            # Using the current method, get the keypoints for each frame
            im_k = detector.detect(im)
            im_k, im_d = descriptor.compute(im, im_k) 

            # Time: How long it took to do keypoint detection on this frame
            frame_kp_detect_time = time.time()
            frametimes['kp_detect_time'].append(frame_kp_detect_time - frame_open_time)

            # BFMatcher with norm type parameter dependant on the keypoint method
            if method == 'ORB' or method == 'BRISK':
                bf = cv2.BFMatcher(normType = cv2.NORM_HAMMING)
            else:
                bf = cv2.BFMatcher()

            matches = bf.knnMatch(im_d, t_d, k=2)
            # Time: How long it took to do the keypoint matching on this frame
            frame_match_time = time.time()            
            frametimes['match_time'].append(frame_match_time - frame_kp_detect_time)

            # Apply ratio test
            for m,n in matches:
                if m.distance < 0.75*n.distance:
                    # Get coordinate of the match
                    m_x = int(im_k[m.queryIdx].pt[0])
                    m_y = int(im_k[m.queryIdx].pt[1])
                    good_matches.append(m)

                    # Increment totalmatches and f_totalmatches
                    totalmatches += 1
                    f_totalmatches += 1

                    # Increment correctmatches and f_correctmatches if this match agrees with the ground truth
                    if x-10 <= m_x <= x2+10 and y-10 <= m_y <= y2+10:
                        correctmatches += 1
                        f_correctmatches += 1

            # append success rate for that frame to f_successes
            f_successes.append(float(f_correctmatches)/float(f_totalmatches))

            if visualize: #make sure that these inputs are passed in correctly!! otherwise nonsensical visualizations will happen.
                kpvisualize(img_t = trainimg, 
                            img_q = imname, 
                            box = row[1:5], 
                            good_matches = good_matches, 
                            t_k = t_k, 
                            q_k = im_k)

                # Time: How long it took to do the ratio test and visualize the matches on this frame 
                # i.e. if the visualize option is True.
                frame_vis_time = time.time()
                frametimes['vis_time'].append(frame_vis_time - frame_match_time)
            else:
                frametimes['vis_time'].append(0)

            frame_end_time = time.time()
            frame_total_times.append(frame_end_time - frame_start_time)
            frametimes['start_to_end_time'].append(frame_end_time - frame_start_time)

        method_end_time = time.time()
        method_total_time = method_end_time - method_start_time
        method_start_time = 0
        method_end_time = 0

        # Getting the averages for the frametimes dictionary values
        for key in frametimes:
            value = frametimes[key]
            frametimes[key] = sum(value) / float(len(value))

        # Compute success ratios for all the rows for this particular method
        # successes[method] = [correctmatches, totalmatches, float(correctmatches)/float(totalmatches)*100, method_total_time, frametimes]
        successes[method] = f_successes

    # Return dictionary of method: success ratio
    return successes

def print_dm_res(d, csv_str):
    "For rapid prototyping purposes: Printing out the metrics in an easy-to-read fashion."
    for key in d:
        print "%s with the %s snippet" % (key, csv_str)
        current = d[key]
        print current
        # print "\tcorrectmatches: %d" % current[]
        # print "\ttotalmatches: %d" % current[]
        # print "\t%d%% accuracy" % current[0] 
        # print "\tmethod total runtime: %f seconds" % current[]
        # print current[1]
        # frametimes = current[]
        # print "\t\taverage start to end time per frame: %f seconds" % frametimes['start_to_end_time']
        # print "\t\taverage image opening time per frame: %f seconds" % frametimes['open_time']
        # print "\t\taverage keypoint detection time per frame: %f seconds" % frametimes['kp_detect_time']
        # print "\t\taverage feature matching time per frame: %f seconds" % frametimes['match_time']
        # print "\t\taverage visualization time per frame: %f seconds" % frametimes['vis_time']
        print '\n'

if __name__ == '__main__':

    catfoodcsv = ['catfood.csv', 'catfood-a-long.csv', 'catfood-a-short.csv', 'catfood-r-long.csv', 'catfood-r-short.csv']
    cerealcsv = ['cereal.csv', 'cereal-a-long.csv', 'cereal-a-short.csv']
    cookiecsv = ['cookie.csv', 'cookie-a-long.csv', 'cookie-a-short.csv']
    
    inputs = {'cookie':['./OT-res/KP-detect/cookie/cookie-train.jpg', cookiecsv],
              'cereal':['./OT-res/KP-detect/cereal/cereal-train.jpg', cerealcsv], 
              'catfood':['./OT-res/KP-detect/catfood/catfood-train.jpg', catfoodcsv]} 

    for v in inputs:       
        print v
        for x in range(len(inputs[v][1])):
            try:
                res = compare_dm(videoname = v, 
                               trainimg = './OT-res/KP-detect/%s/%s-train.jpg' % (v, inputs[v][1][x][:-4]),
                               gt_csv = inputs[v][1][x],
                               visualize = False)

                pickle.dump(res, open("./OT-res/pickles/p2/%s.p" % (inputs[v][1][x][:-4]), "wb"))
                print res
                # print_dm_res(res, inputs[v][1][x])
            except:
                pass