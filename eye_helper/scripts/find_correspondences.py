#!/usr/bin/env python

import cPickle as pickle
import rospy
import numpy as np
from math import sqrt, pi
from matplotlib import pyplot as plt
from tf.transformations import euler_from_matrix, euler_matrix
import cv2
import time

class FindCorrespondences(object):
    def __init__(self, pickle_file):
        f = open(pickle_file, 'r')
        self.combined_data = pickle.load(f)
        f.close()
        rospy.init_node('find_correspondences')
        self.calc_affinity_map()
        self.im1 = None
        self.im2 = None
        self.curr_im1_idx = -1
        self.curr_im2_idx = -1

        self.im1_pts = []
        self.im2_pts = []

        cv2.namedWindow('corr_map')
        cv2.namedWindow('im1')
        cv2.namedWindow('im2')

        cv2.setMouseCallback('corr_map',self.handle_click)
        cv2.setMouseCallback('im1',self.handle_click_im, 'im1')
        cv2.setMouseCallback('im2',self.handle_click_im, 'im2')

        self.image_name = 'compressed_fisheye_img'

    def handle_click_im(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if param == 'im1':
                self.im1_pts.append((x, y))
            elif param == 'im2':
                self.im2_pts.append((x, y))

    def update_images(self):
        if self.image_name == 'compressed_fisheye_img':
            self.im1 = cv2.imdecode(self.combined_data[self.curr_im1_idx][self.image_name], flags=cv2.CV_LOAD_IMAGE_UNCHANGED)
            self.im2 = cv2.imdecode(self.combined_data[self.curr_im2_idx][self.image_name], flags=cv2.CV_LOAD_IMAGE_UNCHANGED)
        else:
            self.im1 = cv2.resize(cv2.imdecode(self.combined_data[self.curr_im1_idx][self.image_name], flags=cv2.CV_LOAD_IMAGE_UNCHANGED), (1280/2, 720/2))
            self.im2 = cv2.resize(cv2.imdecode(self.combined_data[self.curr_im2_idx][self.image_name], flags=cv2.CV_LOAD_IMAGE_UNCHANGED), (1280/2, 720/2))
        im1_depth_cloud = self.combined_data[self.curr_im1_idx]['odom_to_fisheye_camera'].dot(self.combined_data[self.curr_im1_idx]['odom_points'].T)
        im2_depth_cloud = self.combined_data[self.curr_im2_idx]['odom_to_fisheye_camera'].dot(self.combined_data[self.curr_im2_idx]['odom_points'].T)
        im1_projected, _ = cv2.projectPoints(im1_depth_cloud[[1,2,0],:].T,
                                             (0,0,0),
                                             (0,0,0),
                                             self.combined_data[self.curr_im1_idx]['K']['fisheye_undistorted'],
                                             self.combined_data[self.curr_im1_idx]['D']['fisheye_undistorted'])
        im2_projected, _ = cv2.projectPoints(im2_depth_cloud[[1,2,0],:].T,
                                             (0,0,0),
                                             (0,0,0),
                                             self.combined_data[self.curr_im2_idx]['K']['fisheye_undistorted'],
                                             self.combined_data[self.curr_im2_idx]['D']['fisheye_undistorted'])

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
            self.update_images()


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

        for i in range(len(self.combined_data)):
            curr_distance_traveled = 0
            last_x, last_y = self.combined_data[i]['x'], self.combined_data[i]['y']
            for j in range(i + 1, len(self.combined_data)):
                print (i,j)
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
                stacked = np.vstack((np.divide(i_depth_cloud_at_j[1,:], i_depth_cloud_at_j[0,:]),
                                     np.divide(i_depth_cloud_at_j[2,:], i_depth_cloud_at_j[0,:]),
                                     np.ones((i_depth_cloud_at_j.shape[1],))))
                quick_and_dirty_i_at_j = self.combined_data[i]['K']['color_camera'].dot(stacked)

                stacked = np.vstack((np.divide(j_depth_cloud_at_i[1,:], j_depth_cloud_at_i[0,:]),
                                     np.divide(j_depth_cloud_at_i[2,:], j_depth_cloud_at_i[0,:]),
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
        elif im_name == 'im2':
            points = self.im2_pts
        else:
            return

        for i, xy in enumerate(points):
            cv2.circle(image, xy, 3, colors[i%len(colors)], -1)

    def run(self):
        # Todo: allow stepping through on one side or the other with arrow keys
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            expected_joint_landmarks_copy = cv2.cvtColor(self.expected_joint_landmarks.astype(np.float32),
                                                         cv2.COLOR_GRAY2BGR)
            cv2.circle(expected_joint_landmarks_copy, (int(self.curr_im1_idx), int(self.curr_im2_idx)), 3, (0, 0, 255), -1)

            cv2.imshow('corr_map', expected_joint_landmarks_copy)
            if self.im1 != None:
                """ don't use yaw since we will correct this later
                    undo any pitch we see in the image
                    roll doesn't seem to be properly represented so we are doing some weird things
                    to make everything look right. """
                roll, pitch, yaw = euler_from_matrix(self.combined_data[self.curr_im1_idx]['depth_camera_to_odom'])
                rotation = euler_matrix(-pitch, 0, roll-pi)
                rotation2 = euler_matrix(-pi/2, 0, 0)

                K = self.combined_data[self.curr_im1_idx]['K']['fisheye_undistorted']

                H = K.dot(rotation[0:3,0:3].dot(np.linalg.inv(K)))
                warped = cv2.warpPerspective(self.im1, H, (self.im1.shape[1], self.im1.shape[0]))

                # change the focal length so we can see more of the ground and ceiling
                K2 = np.copy(K)
                K2[0,0] *= 0.1
                K2[1,1] *= 0.1
                H2 = K2.dot(rotation2[0:3,0:3].dot(rotation[0:3,0:3]).dot(np.linalg.inv(K)))
                warped_downward = cv2.warpPerspective(self.im1, H2, (self.im1.shape[1], self.im1.shape[0]), flags=cv2.INTER_CUBIC)

                self.draw_points(warped, 'im1')
                cv2.imshow('im1', warped)
            if self.im2 != None:
                """ don't use yaw since we will correct this later
                    undo any pitch we see in the image
                    roll doesn't seem to be properly represented so we are doing some weird things
                    to make everything look right. """
                roll, pitch, yaw = euler_from_matrix(self.combined_data[self.curr_im2_idx]['depth_camera_to_odom'])

                rotation = euler_matrix(-pitch, 0, roll-pi)
                K = self.combined_data[self.curr_im2_idx]['K']['fisheye_undistorted']
                H = K.dot(rotation[0:3,0:3].dot(np.linalg.inv(K)))
                warped = cv2.warpPerspective(self.im2, H, (self.im2.shape[1], self.im2.shape[0]), flags=cv2.INTER_CUBIC)
                self.draw_points(warped, 'im2')
                cv2.imshow('im2', warped)

            k = cv2.waitKey(5)
            if k % 256 == 81:
                # left arrow
                self.curr_im1_idx -= 1
                self.curr_im1_idx = max(0, self.curr_im1_idx)
                self.curr_im1_idx = min(len(self.combined_data) - 1, self.curr_im1_idx)
                self.update_images()
            elif k % 256 == 83:
                # right arrow
                self.curr_im1_idx += 1
                self.curr_im1_idx = max(0, self.curr_im1_idx)
                self.curr_im1_idx = min(len(self.combined_data) - 1, self.curr_im1_idx)
                self.update_images()
            elif k % 256 == 84:
                # up arrow
                self.curr_im2_idx += 1
                self.curr_im2_idx = max(0, self.curr_im2_idx)
                self.curr_im2_idx = min(len(self.combined_data) - 1, self.curr_im2_idx)
                self.update_images()
            elif k % 256 == 82:
                # down arrow
                self.curr_im2_idx -= 1
                self.curr_im2_idx = max(0, self.curr_im2_idx)
                self.curr_im2_idx = min(len(self.combined_data) - 1, self.curr_im2_idx)
                self.update_images()
            elif k % 256 == 10:
                print "pressed enter"
            r.sleep()

if __name__ == '__main__':
    node = FindCorrespondences('./combo_sets.pickle')
    node.run()