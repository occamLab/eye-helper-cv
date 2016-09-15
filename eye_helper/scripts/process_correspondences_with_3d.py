#!/usr/bin/env python

"""
TODO + Takeaways (start with issue of timing and precision... trying to design around that seems to be limiting precision):
    (1) There doesn't seem to be too much difference between rigid transform method and homography method
    (2) Choosing line segments with different depths doesn't work using a naive implementation (just adding
        additional correspondences).  It may be worth trying to derive the rigid transform when the depth of
        the line segments is allowed to be inferred (this could wind up being very simple).  Rotation should not 
        be a problem (with the exception of how the points are weighted)
*** (3) We need to be really careful about getting the pose data.  We should make sure the pose data is as up to 
        date as synched as possible with respect to the fisheye images.  Currently we are synching to the depth
        camera images, which is not ideal.  Further, if we enable low_latency_imu_integration for pose this could
        further increase the accuracy of the pose.
    (4) Check out the pose using Tango API of the fisheye camera (is it possible it is different than depth camera?)
    (5) Once we have more confidence in the precision of  various components we can try to refine correspondences based on this.
    (6) When correcting pitch, a cue we are not using is to make the depths consistent across the two views.
    (7) There are some very interesting new options in the tango config.  For instance there is a drift corrected pose mode that
        is different than area learning.  The config also indicates 100 Hz pose updates.  Additionally, the java version says 
        that there is a 3-DoF fall back when 6-dof is not available (the key appears to be "config_experimental_3dof_fallback")
"""

from scipy.stats.mstats import gmean
import cPickle as pickle
from scipy.linalg import inv, qr
import numpy as np
from tf.transformations import euler_matrix, euler_from_matrix
import cv2

def qr_null(A, tol=None):
    Q, R, P = qr(A.T, mode='full', pivoting=True)
    tol = np.finfo(R.dtype).eps if tol is None else tol
    rnk = min(A.shape) - np.abs(np.diag(R))[::-1].searchsorted(tol)
    return Q[:, rnk:].conj()

def rigid_transform_2D(A, B):
    A, B = np.matrix(A), np.matrix(B)
    # TODO: first possibility, we are computing transform in the wrong direction
    A, B = B, A
   # A = np.hstack((-A[:,1],A[:,0]))
    #B = np.hstack((-B[:,1],B[:,0]))

    # TODO: second possibility is we need to apply coordinate sysetm change before calculating transform
    assert len(A) == len(B)

    N = A.shape[0]; # total points

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    
    # centre the points
    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    H = np.transpose(AA) * BB

    U, S, Vt = np.linalg.svd(H)

    R = Vt.T * U.T

    # special reflection case
    if np.linalg.det(R) < 0:
       Vt[1,:] *= -1
       R = Vt.T * U.T

    # TODO: check into this... could be wrong
    t = -R*centroid_A.T + centroid_B.T
    # we want the translation expressed in the original coordinates
    #t = centroid_B.T - centroid_A.T

    return np.asarray(R), np.asarray(t)

class CorrespondenceProcessor(object):
    def __init__(self, combo_file="combo_sets.pickle", correspondences_file="./correspondences/clicked_points.pickle"):
        f = open(combo_file, 'r')
        self.combined_data = pickle.load(f)
        f.close()
        f = open(correspondences_file)
        self.correspondences = pickle.load(f)
        self.planes = pickle.load(f)
        self.plane_inliers = pickle.load(f)
        f.close()
        self.get_transforms()

    def get_transforms(self):
        self.solutions = {}
        self.estimated = {}
        # Using the 2 point w plane algorithm from: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7439820
        for im_idx_pair, (im1_pts, im2_pts, im1_pts_3d, im2_pts_3d) in self.correspondences.items():
            if im1_pts_3d.shape[0] and im2_pts_3d.shape[0]:
                # TODO: use transforms instead of this due to timing inaccuracies
                delta_theta = self.combined_data[im_idx_pair[1]]['theta'] - self.combined_data[im_idx_pair[0]]['theta']
                delta_x = self.combined_data[im_idx_pair[1]]['x'] - self.combined_data[im_idx_pair[0]]['x']
                delta_y = self.combined_data[im_idx_pair[1]]['y'] - self.combined_data[im_idx_pair[0]]['y']

                relative_basis = np.asarray([[np.cos(self.combined_data[im_idx_pair[0]]['theta']), -np.sin(self.combined_data[im_idx_pair[0]]['theta'])],
                                             [np.sin(self.combined_data[im_idx_pair[0]]['theta']), np.cos(self.combined_data[im_idx_pair[0]]['theta'])]]).T
                trans_estimated = relative_basis.dot(np.asarray([delta_x, delta_y]))
                newR, newT = rigid_transform_2D(im1_pts_3d[:, :2], im2_pts_3d[:, :2])

                newT3D = -np.vstack((newT, np.asarray(0)))
                tmpR = np.zeros((3,3))
                tmpR[0:2, 0:2] = newR
                tmpR[2,2] = 1
                _, _, newR3D = euler_from_matrix(tmpR)

                self.solutions[im_idx_pair] = newT3D, newR3D
                self.estimated[im_idx_pair] = (trans_estimated, delta_theta)