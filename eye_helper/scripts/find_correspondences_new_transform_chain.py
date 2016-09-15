#!/usr/bin/env python

import cPickle as pickle
import rospy
import numpy as np
from math import sqrt, pi, asin
from matplotlib import pyplot as plt
from tf.transformations import euler_from_matrix, euler_matrix
import cv2
from scipy.optimize import brenth
import time
import pcl
import os



def get_point_on_plane(model, pixel, K):
    """ model is the coefficients of a plan in Hessian normal form
        pixel is a pixel coordinate (x, y) in homogeneous coordinates
        K is the matrix of camera intrinsics

        returns the 3d coordinates of the point on the plane that would be imaged at the specified pixel
    """
    p = np.linalg.inv(K).dot(pixel)
    depth = -model[3]/((model[0]*p[0]+model[1]*p[1]+model[2]))

    test = np.asarray([p[0]*depth, p[1]*depth, depth])
    test_projected, _ = cv2.projectPoints(test[:, np.newaxis].T, (0,0,0), (0,0,0), K, np.asarray([0.0, 0.0, 0.0, 0.0, 0.0]))
    #print "on_plane", test[0]*model[0] + test[1]*model[1] + test[2]*model[2] + model[3]
    return np.asarray([p[0]*depth, p[1]*depth, depth])

def correct_roll(plane, three_d, residual_roll, K=None, pixels=None):
    v = np.asarray([plane[0], 0, plane[2]])
    v = v / np.linalg.norm(v)
    v_n = np.asarray([-v[2], v[1], v[0]])

    corrected = cv2.Rodrigues(-v_n*residual_roll)[0].dot(three_d.T)
    cloud = pcl.PointCloud(np.asarray(corrected.T, dtype=np.float32))
    seg = cloud.make_segmenter()
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(1)
    indices, model = seg.segment()
    if pixels != None:
        T = K.dot(cv2.Rodrigues(-v_n*residual_roll)[0]).dot(np.linalg.inv(K))
        pixels_corrected = [T.dot(np.asarray([p[0], p[1], 1])) for p in pixels]
        pixels_corrected = [(p[0]/p[2], p[1]/p[2]) for p in pixels_corrected]
    else:
        pixels_corrected = None
    return model[1], model, corrected, pixels_corrected

def correct_pitch(plane, three_d, residual_pitch, K, pixels):
    v = np.asarray([plane[0], 0, plane[2]])
    v = v / np.linalg.norm(v)
    v_n = np.asarray([-v[2], v[1], v[0]])

    corrected = cv2.Rodrigues(-v*residual_pitch)[0].dot(three_d.T)
    #corrected = euler_matrix(0, -residual_pitch, 0)[0:3,0:3].dot(three_d.T)
    cloud = pcl.PointCloud(np.asarray(corrected.T, dtype=np.float32))
    seg = cloud.make_segmenter()
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(1)
    indices, model = seg.segment()
    T = K.dot(cv2.Rodrigues(-v*residual_pitch)[0]).dot(np.linalg.inv(K))
    pixels_corrected = [T.dot(np.asarray([p[0], p[1], 1])) for p in pixels]
    corrected_3d_pixels = np.asarray([get_point_on_plane(model, np.asarray([p[0]/p[2], p[1]/p[2], 1.0]), K) for p in pixels_corrected])
    pixels_corrected = [(p[0]/p[2], p[1]/p[2]) for p in pixels_corrected]
    return corrected_3d_pixels[0,1] - corrected_3d_pixels[1,1], model, corrected_3d_pixels, pixels_corrected

class FindCorrespondences(object):
    def __init__(self, pickle_file, correspondences_pickle_file):
        self.project_depth_clouds = False
        self.scale_factor = 1

        f = open(pickle_file, 'r')
        self.combined_data = pickle.load(f)
        f.close()
        rospy.init_node('find_correspondences')
        self.calc_affinity_map()
        self.correspondences = {}
        self.planes = {}
        self.plane_inliers = {}

        if os.path.exists(correspondences_pickle_file):
            f = open(correspondences_pickle_file)
            self.correspondences = pickle.load(f)
            self.planes = pickle.load(f)
            self.plane_inliers = pickle.load(f)
            f.close()

        self.correspondences_pickle_file = correspondences_pickle_file
        self.clear_correspondences(False)

        self.im1 = None
        self.im2 = None
        self.curr_im1_idx = -1
        self.curr_im2_idx = -1

        cv2.namedWindow('corr_map')
        cv2.namedWindow('im1')
        cv2.namedWindow('im2')

        cv2.setMouseCallback('corr_map',self.handle_click)
        cv2.setMouseCallback('im1',self.handle_click_im, 'im1')
        cv2.setMouseCallback('im2',self.handle_click_im, 'im2')

        #self.image_name = 'color_camera'
        self.image_name = 'fisheye_undistorted'

    def update_3d_points(self):
        do_correction = False
        if self.im1_plane != None:
            K = self.combined_data[self.curr_im1_idx]['K'][self.image_name]

            if self.im1_pts:
                if do_correction:
                    f = lambda x: correct_roll(self.im1_plane, self.im1_plane_inliers[1], x, K, self.im1_pts)[0]
                    estimated_residual_roll = asin(self.im1_plane[1])
                    print "estimated_residual roll", estimated_residual_roll
                    optimized_roll = brenth(f, -estimated_residual_roll*2, estimated_residual_roll*2)
                    print "optimized_roll im1", optimized_roll

                    _, corrected_model, corrected_3d, corrected_pixels = correct_roll(self.im1_plane, self.im1_plane_inliers[1], optimized_roll, K, self.im1_pts)
                    print "roll corrected model", corrected_model
                    g = lambda x: correct_pitch(corrected_model, corrected_3d.T, x, K, corrected_pixels)[0]
                    optimized_pitch = brenth(g, -.5, .5)
                    print "optimized pitch im1", optimized_pitch

                    _, final_model, corrected_3d_pixels, corrected_pixels = correct_pitch(corrected_model, corrected_3d.T, optimized_pitch, K, corrected_pixels)
                    print "final_model im1", final_model
                    print 'original 3d im1', np.asarray([get_point_on_plane(self.im1_plane, np.asarray([p[0], p[1], 1.0]), K) for p in self.im1_pts])
                    print "final 3d im1", corrected_3d_pixels
                    self.im1_pts_3d = corrected_3d_pixels
                    # TODO: can't overwrite this since we need to maintain the uncorrected version.  Need to write this to the pixel file though.
                    #self.im1_pts = corrected_pixels
                else:
                    self.im1_pts_3d = np.asarray([get_point_on_plane(self.im1_plane, np.asarray([p[0], p[1], 1.0]), K) for p in self.im1_pts])
            else:
                self.im1_pts_3d = np.zeros((0, 3))
        if self.im2_plane != None:
            K = self.combined_data[self.curr_im2_idx]['K'][self.image_name]
            if self.im2_pts:
                if do_correction:
                    f = lambda x: correct_roll(self.im2_plane, self.im2_plane_inliers[1], x, K, self.im2_pts)[0]
                    estimated_residual_roll = asin(self.im2_plane[1])
                    optimized_roll = brenth(f, -estimated_residual_roll*2, estimated_residual_roll*2)
                    print "optimized_roll im2", optimized_roll

                    _, corrected_model, corrected_3d, corrected_pixels = correct_roll(self.im2_plane, self.im2_plane_inliers[1], optimized_roll, K, self.im2_pts)
                    print "roll corrected model", corrected_model
                    g = lambda x: correct_pitch(corrected_model, corrected_3d.T, x, K, corrected_pixels)[0]
                    optimized_pitch = brenth(g, -.5, .5)
                    print "optimized pitch im2", optimized_pitch
                    _, final_model, corrected_3d_pixels, corrected_pixels = correct_pitch(corrected_model, corrected_3d.T, optimized_pitch, K, corrected_pixels)
                    print "final_model im2", final_model
                    print 'original 3d im2', np.asarray([get_point_on_plane(self.im2_plane, np.asarray([p[0], p[1], 1.0]), K) for p in self.im2_pts])
                    print "final 3d im2", corrected_3d_pixels
                    self.im2_pts_3d = corrected_3d_pixels
                    # TODO: can't overwrite this since we need to maintain the uncorrected version.  Need to write this to the pickle file though.
                    #self.im2_pts = corrected_pixels
                else:
                    self.im2_pts_3d = np.asarray([get_point_on_plane(self.im2_plane, np.asarray([p[0], p[1], 1.0]), K) for p in self.im2_pts])
            else:
                self.im2_pts_3d = np.zeros((0, 3))


    def correct_roll_and_pitch(self, transformation_matrix, K=None, im=None, points=None, pixel_coordinates=None):
        """ Corrects for roll and pitch by warping the specified image and rotating
            the specified points (3d points in non-homogeneous coordinates) and
            pixel_coordinates (2d points in non-homogeneous coordinates).  The specified
            transformation_matrix is a 4x4 matrix that operates on homogeneous 3d points. 
            K is the camera intrinsics matrix.

            TODO: currently we are correcting the pitch of the transform due to an inaccurate
                  static transform between device and depth_camera frame in most of the
                  recorded bag files
            """
        im_warped, points_rotated, pixel_coordinates_rotated = None, None, None
        pitch, roll, yaw = euler_from_matrix(transformation_matrix)
        pitch = pitch + np.pi/2

        rotation = euler_matrix(pitch, 0, roll)
        # D stands for the desired axes after applying the transform A
        D = np.asarray([[0, 0, 1],[-1, 0, 0],[0, -1, 0]])
        rotation = np.linalg.inv(D).dot(transformation_matrix[:-1,:-1])
        rotation = np.concatenate((rotation, np.asarray([0, 0, 0])[:,np.newaxis]), axis=1)
        rotation = np.concatenate((rotation, np.asarray([[0, 0, 0, 1]])), axis=0)

        # undo correction of the image about the y-axis (yaw)
        rotation = euler_matrix(0, -euler_from_matrix(rotation, axes='syxz')[0],  0).dot(rotation)

        if K != None:
            H = K.dot(rotation[0:3,0:3].dot(np.linalg.inv(K)))
        if im != None:
            im_warped = cv2.warpPerspective(im, H, (self.im1.shape[1], self.im1.shape[0]))
        if points != None:
            points_rotated = rotation[0:3, 0:3].dot(points)
        if pixel_coordinates != None:
            pixel_coordinates_homogeneous = np.concatenate((pixel_coordinates, np.ones((pixel_coordinates.shape[0],1))), axis=1)
            pixel_coordinates_rotated = H.dot(pixel_coordinates_homogeneous.T)
            pixel_coordinates_rotated = pixel_coordinates_rotated[:,:2].T
        return im_warped, points_rotated, pixel_coordinates_rotated

    def handle_click_im(self, event, x, y, flags, param):
        x, y = x/self.scale_factor, y/self.scale_factor
        if event == cv2.EVENT_LBUTTONDOWN:
            if param == 'im1':
                self.im1_pts.append((x, y))
            elif param == 'im2':
                self.im2_pts.append((x, y))
            self.update_3d_points()
        elif event == cv2.EVENT_RBUTTONDOWN:
            if param == 'im1':
                im_idx = self.curr_im1_idx
            else:
                im_idx = self.curr_im2_idx
            if self.image_name == 'fisheye_undistorted':
                im_depth_cloud = self.combined_data[im_idx]['odom_to_fisheye_camera'].dot(self.combined_data[im_idx]['odom_points'].T)
                transform = self.combined_data[im_idx]['fisheye_camera_to_odom']
            else:
                im_depth_cloud = self.combined_data[im_idx]['odom_to_depth_camera'].dot(self.combined_data[im_idx]['odom_points'].T)
                transform = self.combined_data[im_idx]['depth_camera_to_odom']

            K = self.combined_data[im_idx]['K'][self.image_name]
            D = self.combined_data[im_idx]['D'][self.image_name]

            # TODO: this is where you need to make any adjustments in axes conventions.  Currently, I don't think there are any.
            depth_camera_axes = im_depth_cloud[[0,1,2],:]

            _, depth_rotated, _ = self.correct_roll_and_pitch(transformation_matrix=transform, points=depth_camera_axes)

            im_projected_rotated, _ = cv2.projectPoints(depth_rotated.T, (0,0,0), (0,0,0), K, D)
            im_projected_rotated = im_projected_rotated.squeeze().T

            depth_rotated_filtered = np.asarray([depth_rotated[:,i] for i in range(im_projected_rotated.shape[1]) if np.linalg.norm(im_projected_rotated[:,i] - np.asarray([x,y])) < 30])
            im_projected_rotated_filtered = np.asarray([im_projected_rotated[:,i] for i in range(im_projected_rotated.shape[1]) if np.linalg.norm(im_projected_rotated[:,i] - np.asarray([x,y])) < 30]).T
            if len(depth_rotated_filtered.shape) == 2:
                cloud = pcl.PointCloud(np.asarray(depth_rotated_filtered, dtype=np.float32))
                seg = cloud.make_segmenter()
                seg.set_optimize_coefficients(True)
                seg.set_model_type(pcl.SACMODEL_PLANE)

                seg.set_method_type(pcl.SAC_RANSAC)
                seg.set_distance_threshold(.01)     # used to be .01
                indices, model = seg.segment()
                print 'fit model', model
                inliers = im_projected_rotated_filtered[:, indices]
                inliers_3d = depth_rotated_filtered[indices, :]
                inliers = inliers, inliers_3d
                if param == 'im1':
                    self.im1_plane_inliers = inliers
                    self.im1_plane = model
                else:
                    self.im2_plane_inliers = inliers
                    self.im2_plane = model
                self.update_3d_points()

    def update_images(self):
        if self.image_name == 'fisheye_undistorted':
            self.im1 = cv2.imdecode(self.combined_data[self.curr_im1_idx][self.image_name], flags=cv2.CV_LOAD_IMAGE_UNCHANGED)
            self.im2 = cv2.imdecode(self.combined_data[self.curr_im2_idx][self.image_name], flags=cv2.CV_LOAD_IMAGE_UNCHANGED)
            im1_depth_cloud = self.combined_data[self.curr_im1_idx]['odom_to_fisheye_camera'].dot(self.combined_data[self.curr_im1_idx]['odom_points'].T)
            im2_depth_cloud = self.combined_data[self.curr_im2_idx]['odom_to_fisheye_camera'].dot(self.combined_data[self.curr_im2_idx]['odom_points'].T)
        else:
            self.im1 = cv2.imdecode(self.combined_data[self.curr_im1_idx][self.image_name], flags=cv2.CV_LOAD_IMAGE_UNCHANGED)
            self.im2 = cv2.imdecode(self.combined_data[self.curr_im2_idx][self.image_name], flags=cv2.CV_LOAD_IMAGE_UNCHANGED)
            im1_depth_cloud = self.combined_data[self.curr_im1_idx]['odom_to_depth_camera'].dot(self.combined_data[self.curr_im1_idx]['odom_points'].T)
            im2_depth_cloud = self.combined_data[self.curr_im2_idx]['odom_to_depth_camera'].dot(self.combined_data[self.curr_im2_idx]['odom_points'].T)

        if self.project_depth_clouds: 
            im1_depth_cloud_camera_coords = np.copy(im1_depth_cloud)
            im1_depth_cloud_camera_coords = im1_depth_cloud_camera_coords[[0,1,2],:]
            # im1_depth_cloud_camera_coords[[0,1],:] *= -1
            im1_projected, _ = cv2.projectPoints(im1_depth_cloud_camera_coords.T,
                                                 (0,0,0),
                                                 (0,0,0),
                                                 self.combined_data[self.curr_im1_idx]['K'][self.image_name],
                                                 self.combined_data[self.curr_im1_idx]['D'][self.image_name])
            im2_depth_cloud_camera_coords = np.copy(im2_depth_cloud)
            im2_depth_cloud_camera_coords = im2_depth_cloud_camera_coords[[0,1,2],:]
            # im2_depth_cloud_camera_coords[[0,1],:] *= -1

            im2_projected, _ = cv2.projectPoints(im2_depth_cloud_camera_coords.T,
                                                 (0,0,0),
                                                 (0,0,0),
                                                 self.combined_data[self.curr_im2_idx]['K'][self.image_name],
                                                 self.combined_data[self.curr_im2_idx]['D'][self.image_name])

            for i in range(im1_projected.shape[0]):
                try:
                    self.im1[int(im1_projected[i,0,1]), int(im1_projected[i,0,0]),:] = 255*0.2*np.ones(3,) + 0.8*self.im1[int(im1_projected[i,0,1]), int(im1_projected[i,0,0]),:]
                except:
                    pass
            for i in range(im2_projected.shape[0]):
                try:
                    self.im2[int(im2_projected[i,0,1]), int(im2_projected[i,0,0]),:] = 255*0.2*np.ones(3,) + 0.8*self.im2[int(im2_projected[i,0,1]), int(im2_projected[i,0,0]),:]
                except:
                    pass

    def handle_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.curr_im1_idx = x
            self.curr_im2_idx = y
            self.clear_correspondences()
            self.update_images()

    def clear_correspondences(self, reload_saved_correspondences=True):
        self.im1_pts = []
        self.im2_pts = []
        self.im1_plane_inliers = (np.zeros((2,0)), np.zeros((0,3)))
        self.im2_plane_inliers = (np.zeros((2,0)), np.zeros((0,3)))
        self.im1_pts_3d = np.zeros((0, 3))
        self.im2_pts_3d = np.zeros((0, 3))
        self.im1_plane = None
        self.im2_plane = None

        if reload_saved_correspondences:
            self.im1_pts, self.im2_pts, self.im1_pts_3d, self.im2_pts_3d = self.correspondences.get((self.curr_im1_idx, self.curr_im2_idx), [[], [], np.zeros((0, 3)), np.zeros((0, 3))])
            self.im1_plane, self.im2_plane = self.planes.get((self.curr_im1_idx, self.curr_im2_idx), (None, None))
            self.im1_plane_inliers, self.im2_plane_inliers = self.plane_inliers.get((self.curr_im1_idx, self.curr_im2_idx), ((np.zeros((2,0)), np.zeros((0,3))), (np.zeros((2,0)), np.zeros((0,3)))))

    def calc_affinity_map(self):
        self.distance_traveled = np.zeros((len(self.combined_data), len(self.combined_data)))
        self.expected_joint_landmarks = np.zeros((len(self.combined_data), len(self.combined_data)))

        # TODO: this could be done much faster using cumulative sum of distances
        for i in range(len(self.combined_data)):
            curr_distance_traveled = 0
            last_x, last_y = self.combined_data[i]['x'], self.combined_data[i]['y']
            for j in range(i + 1, len(self.combined_data)):
                curr_distance_traveled += sqrt((self.combined_data[j]['x'] - last_x)**2 + (self.combined_data[j]['y'] - last_y)**2)
                last_x, last_y = self.combined_data[j]['x'], self.combined_data[j]['y']
                self.distance_traveled[i, j] = curr_distance_traveled

        for i in range(0, len(self.combined_data), 100):
            curr_distance_traveled = 0
            last_x, last_y = self.combined_data[i]['x'], self.combined_data[i]['y']
            for j in range(i + 1, len(self.combined_data)):
                #print (i,j)
                # convert odom points to coordinate system of the other camera
                j_depth_cloud_at_i = self.combined_data[i]['odom_to_depth_camera'].dot(self.combined_data[j]['odom_points'].T)
                i_depth_cloud_at_j = self.combined_data[j]['odom_to_depth_camera'].dot(self.combined_data[i]['odom_points'].T)

                # project each depth cloud on the camera.  Need to reshuffle coordinates due to OpenCV's camera conventions
                # j_projected_at_i, dc = cv2.projectPoints(j_depth_cloud_at_i[[1,2,0],:].T,
                #                                          (0,0,0),
                #                                          (0,0,0),
                #                                          self.combined_data[i]['K'],
                #                                          self.combined_data[i]['D'])

                # i_projected_at_j, dc = cv2.projectPoints(i_depth_cloud_at_j[[1,2,0],:].T,
                #                                          (0,0,0),
                #                                          (0,0,0),
                #                                          self.combined_data[i]['K'],
                #                                          self.combined_data[i]['D'])

                # might not need to do the calculation in both directions
                # ignoring distortion makes this way faster.  Alternatively, could probably implement the distortion faster than the general purpose cv2.projectPoints
                # this is weird to match coordinate system conventions of opencv versus ROS
                stacked = np.vstack((np.divide(-i_depth_cloud_at_j[1,:], i_depth_cloud_at_j[0,:]),
                                     np.divide(-i_depth_cloud_at_j[2,:], i_depth_cloud_at_j[0,:]),
                                     np.ones((i_depth_cloud_at_j.shape[1],))))
                quick_and_dirty_i_at_j = self.combined_data[i]['K']['color_camera'].dot(stacked)

                stacked = np.vstack((np.divide(-j_depth_cloud_at_i[1,:], j_depth_cloud_at_i[0,:]),
                                     np.divide(-j_depth_cloud_at_i[2,:], j_depth_cloud_at_i[0,:]),
                                     np.ones((j_depth_cloud_at_i.shape[1],))))
                quick_and_dirty_j_at_i = self.combined_data[j]['K']['color_camera'].dot(stacked)
                j_at_i_mask = ~np.any(np.vstack((quick_and_dirty_j_at_i[0,:] < -50,
                                                 quick_and_dirty_j_at_i[1,:] < -50,
                                                 quick_and_dirty_j_at_i[0,:] > 1330,
                                                 quick_and_dirty_j_at_i[1,:] > 770,
                                                 j_depth_cloud_at_i[0,:] < 0.0,
                                                 j_depth_cloud_at_i[0,:] > 8.0)),
                                      axis=0)

                i_at_j_mask = ~np.any(np.vstack((quick_and_dirty_i_at_j[0,:] < -50,
                                                 quick_and_dirty_i_at_j[1,:] < -50,
                                                 quick_and_dirty_i_at_j[0,:] > 1330,
                                                 quick_and_dirty_i_at_j[1,:] > 770,
                                                 i_depth_cloud_at_j[0,:] < 0.0,
                                                 i_depth_cloud_at_j[0,:] > 8.0)),
                                      axis=0)

                self.expected_joint_landmarks[i, j] = min(i_at_j_mask.mean(), j_at_i_mask.mean())

    def draw_points(self, image, im_name):
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
        if im_name == 'im1':
            points = self.im1_pts
            inlier_points = self.im1_plane_inliers[0]
            cv2.putText(image, 'Frame: %d' % (self.curr_im1_idx), (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))
        elif im_name == 'im2':
            points = self.im2_pts
            cv2.putText(image, 'Frame: %d' % (self.curr_im2_idx), (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))
            inlier_points = self.im2_plane_inliers[0]
        else:
            return

        for i, xy in enumerate(points):
            cv2.circle(image, (int(xy[0]), int(xy[1])), 3, colors[i%len(colors)], -1)

        # as a sanity check draw the point cloud TODO: remove
        for i in range(inlier_points.shape[1]):
            cv2.circle(image, (int(inlier_points[0,i]), int(inlier_points[1,i])), 2, (255,0,0), 1)

    def run(self):
        # Todo: allow stepping through on one side or the other with arrow keys
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            expected_joint_landmarks_copy = cv2.cvtColor(self.expected_joint_landmarks.astype(np.float32),
                                                         cv2.COLOR_GRAY2BGR)
            cv2.circle(expected_joint_landmarks_copy, (int(self.curr_im1_idx), int(self.curr_im2_idx)), 3, (0, 0, 255), -1)
            cv2.imshow('corr_map', expected_joint_landmarks_copy)
            if self.im1 != None:
                warped, _, _ = self.correct_roll_and_pitch(transformation_matrix=self.combined_data[self.curr_im1_idx]['fisheye_camera_to_odom'],
                                                           K=self.combined_data[self.curr_im1_idx]['K'][self.image_name],
                                                           im=self.im1)

                self.draw_points(warped, 'im1')
                cv2.imshow('im1',cv2.resize(warped, (int(warped.shape[1]*self.scale_factor), int(warped.shape[0]*self.scale_factor))))
            if self.im2 != None:
                warped, _, _ = self.correct_roll_and_pitch(transformation_matrix=self.combined_data[self.curr_im2_idx]['fisheye_camera_to_odom'],
                                                           K=self.combined_data[self.curr_im2_idx]['K'][self.image_name],
                                                           im=self.im2)
                self.draw_points(warped, 'im2')
                cv2.imshow('im2',cv2.resize(warped, (int(warped.shape[1]*self.scale_factor), int(warped.shape[0]*self.scale_factor))))

            k = cv2.waitKey(50)
            if k % 256 == 81:
                # left arrow
                self.curr_im1_idx -= 1
                self.curr_im1_idx = max(0, self.curr_im1_idx)
                self.curr_im1_idx = min(len(self.combined_data) - 1, self.curr_im1_idx)
                self.clear_correspondences()
                self.update_images()
            elif k % 256 == 83:
                # right arrow
                self.curr_im1_idx += 1
                self.curr_im1_idx = max(0, self.curr_im1_idx)
                self.curr_im1_idx = min(len(self.combined_data) - 1, self.curr_im1_idx)
                self.clear_correspondences()
                self.update_images()
            elif k % 256 == 84:
                # up arrow
                self.curr_im2_idx += 1
                self.curr_im2_idx = max(0, self.curr_im2_idx)
                self.curr_im2_idx = min(len(self.combined_data) - 1, self.curr_im2_idx)
                self.clear_correspondences()
                self.update_images()
            elif k % 256 == 82:
                # down arrow
                self.curr_im2_idx -= 1
                self.curr_im2_idx = max(0, self.curr_im2_idx)
                self.curr_im2_idx = min(len(self.combined_data) - 1, self.curr_im2_idx)
                self.clear_correspondences()
                self.update_images()
            elif k % 256 == 10:
                m = min(len(self.im1_pts), len(self.im2_pts))
                self.correspondences[(self.curr_im1_idx, self.curr_im2_idx)] = [self.im1_pts[:m], self.im2_pts[:m], self.im1_pts_3d[:m, :], self.im2_pts_3d[:m, :]]
                self.planes[(self.curr_im1_idx, self.curr_im2_idx)] = [self.im1_plane, self.im2_plane]
                self.plane_inliers[(self.curr_im1_idx, self.curr_im2_idx)] = [self.im1_plane_inliers, self.im2_plane_inliers]

                f = open(self.correspondences_pickle_file, 'wt')
                pickle.dump(self.correspondences, f)
                pickle.dump(self.planes, f)
                pickle.dump(self.plane_inliers, f)
                f.close()

                print "saved correspondences"
            elif k % 256 == 32:
                print "clearing correspondences"
                self.clear_correspondences(False)
            elif k % 256 == ord('d'):
                self.project_depth_clouds = not self.project_depth_clouds
                self.update_images()
            elif k % 256 == ord('-'):
                self.scale_factor = 1
            elif k % 256 == ord('='):
                self.scale_factor = 2
            r.sleep()

if __name__ == '__main__':
    node = FindCorrespondences('./combo_sets.pickle', './correspondences/clicked_points.pickle')
    node.run()