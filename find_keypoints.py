import cv2
import numpy as np
import scipy as sp
import pickle

def find_kp(img, method):
    """
        inputs:
            img -> image you are searching through
            method -> keypoint detection method
        outputs:
            pickle of [keypoints, descriptors]
    """
    im = cv2.imread(img)
    detector = cv2.FeatureDetector_create(method)
    descriptor = cv2.DescriptorExtractor_create(method)     
    im_k = detector.detect(im)
    im_k, im_d = descriptor.compute(im, im_k)

    kps =[]
    for point in im_k:
        kps.append((point.pt[0], point.pt[1]))

    return [kps, im_d]

    
if __name__ == '__main__':
    methods = ['ORB', 'BRISK', 'SIFT', 'SURF']
    datasets = {'cookie': (124, 288), 'catfood': (266, 763), 'cereal': (512, 695)}

    for method in methods:
        for dataset in datasets:
            print 'method: %s, dataset: %s' % (method, dataset)
            for frame in range(datasets[dataset][0], datasets[dataset][1] + 1):
                res = find_kp('../gstore-snippets/%s_snippet/%s_00%d.jpg' % (dataset, dataset, frame),method)
                pickle.dump(res, open('../OT-res/kp_pickles/%s/%s/%s_00%d_keypoints.p' % (dataset, method, dataset, frame), 'wb'))