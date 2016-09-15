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
    A = np.hstack((-A[:,1],A[:,0]))
    B = np.hstack((-B[:,1],B[:,0]))

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
        self.do_downward_rotation = True
        f = open(combo_file, 'r')
        self.combined_data = pickle.load(f)
        f.close()
        f = open(correspondences_file)
        self.correspondences = pickle.load(f)
        self.planes = pickle.load(f)
        self.plane_inliers = pickle.load(f)
        f.close()
        self.get_rotated_points()
        self.get_transforms()

    def get_rotated_points(self):
        self.rotated_correspondences = {}
        rot = euler_matrix(-np.pi/2, 0, 0)[0:3,0:3]

        for (im1_idx, im2_idx), (im1_pts, im2_pts, im1_pts_3d, im2_pts_3d) in self.correspondences.items():
            K1 = self.combined_data[im1_idx]['K']['fisheye_undistorted']
            K2 = self.combined_data[im2_idx]['K']['fisheye_undistorted']

            im1_pts_rotated = []
            im2_pts_rotated = []

            if self.do_downward_rotation:
                im1_pts_3d_rotated = rot.dot(im1_pts_3d.T).T
                im2_pts_3d_rotated = rot.dot(im2_pts_3d.T).T
            else:
                im1_pts_3d_rotated = im1_pts_3d
                im2_pts_3d_rotated = im2_pts_3d

            if self.do_downward_rotation and im1_pts_3d.shape[0]:
                averaged_im1_depth = gmean(im1_pts_3d_rotated[:, 2])
                if not im2_pts_3d.shape[0]:
                    averaged_im2_depth = averaged_im1_depth
                    im2_pts_3d_rotated = np.zeros(im1_pts_3d_rotated.shape)

            if self.do_downward_rotation and im2_pts_3d.shape[0]:
                averaged_im2_depth = gmean(im2_pts_3d_rotated[:, 2])
                if not im1_pts_3d.shape[0]:
                    averaged_im1_depth = averaged_im2_depth
                    im1_pts_3d_rotated = np.zeros(im2_pts_3d_rotated.shape)

            # NOTE: im1_pts and im2_pts are guaranteed to have the same length based on find_correspondences.py
            for i in range(len(im1_pts)):
                im1_pt_homogeneous = np.asarray([im1_pts[i][0], im1_pts[i][1], 1.0], dtype=np.float64)
                im2_pt_homogeneous = np.asarray([im2_pts[i][0], im2_pts[i][1], 1.0], dtype=np.float64)
                im1_pt_normalized = inv(K1).dot(im1_pt_homogeneous)
                im2_pt_normalized = inv(K2).dot(im2_pt_homogeneous)

                # no need to multiply by K1 and K2 since algorithm assumes normalized points
                im1_pt_rotated = rot.dot(im1_pt_normalized)
                im2_pt_rotated = rot.dot(im2_pt_normalized)

                im1_pt_pixel_coordinates = im1_pt_rotated[0]/im1_pt_rotated[2], im1_pt_rotated[1]/im1_pt_rotated[2]
                im2_pt_pixel_coordinates = im2_pt_rotated[0]/im2_pt_rotated[2], im2_pt_rotated[1]/im2_pt_rotated[2]

                # if self.do_downward_rotation and (im1_pts_3d.shape[0] or im2_pts_3d.shape[0]):
                #     # throw away 3d information and use pixels instead.  The 3d information is only used to get the depth.  TODO: not sure if this is more or less accurate than using the 3d info directly.
                #     im1_pts_3d_rotated[i,:] = im1_pt_pixel_coordinates[0]*averaged_im1_depth, im1_pt_pixel_coordinates[1]*averaged_im1_depth, averaged_im1_depth
                #     im2_pts_3d_rotated[i,:] = im2_pt_pixel_coordinates[0]*averaged_im2_depth, im2_pt_pixel_coordinates[1]*averaged_im2_depth, averaged_im2_depth

                im1_pts_rotated.append(im1_pt_pixel_coordinates)
                im2_pts_rotated.append(im2_pt_pixel_coordinates)

            self.rotated_correspondences[(im1_idx, im2_idx)] = [im1_pts_rotated, im2_pts_rotated, im1_pts_3d_rotated, im2_pts_3d_rotated, im1_pts_3d, im2_pts_3d]

    def get_trans_rot(self, sol):
        t = np.asarray([-sol[2], -sol[3], 1-sol[4]])
        R = np.asarray([[sol[0], -sol[1], 0],
                        [sol[1], sol[0], 0],
                        [0, 0, 1]])
        return t, euler_from_matrix(R)[2]

    def get_transforms(self):
        self.solutions = {}
        self.estimated = {}
        # Using the 2 point ground plane algorithm from: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7439820
        for im_idx_pair, (im1_pts, im2_pts, im1_pts_3d, im2_pts_3d, im1_pts_3d_orig, im2_pts_3d_orig) in self.rotated_correspondences.items():
            # each correspondence contributes 2 equations.  We seek to estimate 5 parameters
            A = np.zeros((len(im1_pts)*2, 5))
            b = np.zeros((4,1))
            if len(im1_pts) == 2:
                # todo modify this to handle more correspondences (requires switching between SVD and null space as appropriate)
                for idx, (qi, qj) in enumerate(zip(im1_pts, im2_pts)):
                    # we use non-homogeneous points so consider the z (or 2 component to match the paper) component to be 0
                    xi, yi, wi = qi[0], qi[1], 1
                    xj, yj, wj = qj[0], qj[1], 1
                    # try swapping to see how things change
                    # xj, yj, wj = qi[0], qi[1], 1
                    # xi, yi, wi = qj[0], qj[1], 1
                    # see Equation 11 in the paper cited above
                    A[2*idx, :] = [-wj*yi, -wj*xi, 0, -wi*wj, wi*yj]
                    A[2*idx + 1, :] = [wj*xi, -wj*yi, wi*wj, 0, -wi*xj]
                null_space = qr_null(A)
                solution_1 = null_space / np.sqrt(null_space[0]**2 + null_space[1]**2)
                solution_2 = -null_space / np.sqrt(null_space[0]**2 + null_space[1]**2)

                t_1, R_1 = self.get_trans_rot(solution_1)
                t_2, R_2 = self.get_trans_rot(solution_2)

                if abs(t_1[2]) < abs(t_2[2]):
                    self.solutions[im_idx_pair] = t_1, R_1
                else:
                    self.solutions[im_idx_pair] = t_2, R_2

            delta_theta = self.combined_data[im_idx_pair[1]]['theta'] - self.combined_data[im_idx_pair[0]]['theta']
            delta_x = self.combined_data[im_idx_pair[1]]['x'] - self.combined_data[im_idx_pair[0]]['x']
            delta_y = self.combined_data[im_idx_pair[1]]['y'] - self.combined_data[im_idx_pair[0]]['y']

            relative_basis = np.asarray([[np.cos(self.combined_data[im_idx_pair[0]]['theta']), -np.sin(self.combined_data[im_idx_pair[0]]['theta'])],
                                         [np.sin(self.combined_data[im_idx_pair[0]]['theta']), np.cos(self.combined_data[im_idx_pair[0]]['theta'])]]).T
            trans_estimated = relative_basis.dot(np.asarray([delta_x, delta_y]))
            trans2_estimated = relative_basis.T.dot(np.asarray([delta_x, delta_y]))

            if im1_pts_3d.shape[0] and im2_pts_3d.shape[0]:
                newR, newT = rigid_transform_2D(im1_pts_3d[:, :2], im2_pts_3d[:, :2])
                # TODO: kludge due to sign reversal, need to look into this more
                newT3D = -np.vstack((newT, np.asarray(0)))
                tmpR = np.zeros((3,3))
                tmpR[0:2, 0:2] = newR
                tmpR[2,2] = 1
                _, _, newR3D = euler_from_matrix(tmpR)
            if len(im1_pts) == 2 and len(im2_pts) == 2:
                newR, newT = rigid_transform_2D(np.asarray(im1_pts), np.asarray(im2_pts))
                newT = np.vstack((newT, np.asarray(0)))
                newT = newT / np.linalg.norm(newT)

                tmpR = np.zeros((3,3))
                tmpR[0:2, 0:2] = newR
                tmpR[2,2] = 1
                _, _, newR = euler_from_matrix(tmpR)
                # rotations are identical
                # print "change in rotation", self.solutions[im_idx_pair][1] - newR
                # overwrite with the rigid transform solution
                #self.solutions[im_idx_pair] = newT, newR, newT3D, newR3D
                self.solutions[im_idx_pair] = newT3D, newR3D
                self.estimated[im_idx_pair] = (trans_estimated, delta_theta)

    def get_angle_relationships(self):
        s = {}
        e = {}
        for k in self.solutions:
            s[k] = self.solutions[k][1] % (2*np.pi)
            e[k] = self.estimated[k][2] % (2*np.pi)
        return s, e

    def get_translation_relationships(self):
        s = {}
        e = {}
        for k in self.solutions:
            s_normed = np.sqrt(self.solutions[k][0][0][0]**2 + self.solutions[k][0][1][0]**2)
            # TODO: don't normalize for now
            s_normed = 1
            s[k] = (self.solutions[k][0][0][0]/s_normed, self.solutions[k][0][1][0]/s_normed)
            # convert from camera to world coordinate conventions
            #s[k] = (-s[k][1], s[k][0])
            e_normed = np.sqrt(self.estimated[k][0][0]**2 + self.estimated[k][0][1]**2)
            # TODO: don't normalize for now
            e_normed = 1
            e[k] = (self.estimated[k][0][0]/e_normed, self.estimated[k][0][1]/e_normed)
        return s, e

    def get_projected_lengths(self):
        projected_lengths = {}
        for k in self.rotated_correspondences:
            v1 = np.asarray(self.rotated_correspondences[k][0][0])-np.asarray(self.rotated_correspondences[k][0][1])
            v2 = np.asarray(self.rotated_correspondences[k][1][0])-np.asarray(self.rotated_correspondences[k][1][1])
            projected_lengths[k] = np.linalg.norm(v1), np.linalg.norm(v2)
        return projected_lengths

    def get_downward_image(self, idx):
        roll, pitch, yaw = euler_from_matrix(self.combined_data[idx]['fisheye_camera_to_odom'])
        print "pitch orig", pitch
        pitch -= .039           # correct the pitch
        rotation1 = euler_matrix(0, 0, roll-np.pi)
        rotation2 = euler_matrix(-pitch, 0, 0)
        rotation3 = euler_matrix(-np.pi/2, 0, 0)

        K = self.combined_data[idx]['K']['fisheye_undistorted']

        # change the focal length so we can see more of the ground and ceiling
        K2 = np.copy(K)
        K2[0,0] *= 0.1
        K2[1,1] *= 0.1
        H2 = K2.dot(rotation3[0:3,0:3].dot(rotation2[0:3,0:3].dot(rotation1[0:3,0:3].dot(np.linalg.inv(K)))))
        im = cv2.imdecode(self.combined_data[idx]['fisheye_undistorted'], flags=cv2.CV_LOAD_IMAGE_UNCHANGED)
        warped_downward = cv2.warpPerspective(im, H2, (im.shape[1], im.shape[0]), flags=cv2.INTER_CUBIC)
        return warped_downward, K2

    def visualize_correspondences(self, idx1, idx2):
        warped_down1, K1 = self.get_downward_image(idx1)
        warped_down2, K2 = self.get_downward_image(idx2)

        for i in range(len(self.rotated_correspondences[idx1,idx2][0])):
            v = np.asarray([self.rotated_correspondences[idx1,idx2][0][i][0],
                            self.rotated_correspondences[idx1,idx2][0][i][1],
                            1.0])
            p = K1.dot(v)
            cv2.circle(warped_down1, (int(p[0]), int(p[1])), 3, 255, -1)

            v = np.asarray([self.rotated_correspondences[idx1,idx2][1][i][0],
                            self.rotated_correspondences[idx1,idx2][1][i][1],
                            1.0])
            p = K2.dot(v)
            cv2.circle(warped_down2, (int(p[0]), int(p[1])), 3, 255, -1)

        cv2.imshow('im1', warped_down1)
        cv2.imshow('im2', warped_down2)

        cv2.waitKey(50)