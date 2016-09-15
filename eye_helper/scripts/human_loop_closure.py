#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Pose, Point32
from sensor_msgs.msg import CameraInfo, PointCloud, CompressedImage
import cv2
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from std_msgs.msg import Header
from tf import TransformListener
from threading import Lock
import time
import cPickle as pickle

class DepthImageCreator(object):
    def __init__(self):
        self.depth_image_lock = Lock()
        self.image_list_lock = Lock()
        self.fisheye_image_list_lock = Lock()

        self.image_list = []
        self.fisheye_image_list = []

        self.image_list_max_size = 100
        self.combined_data = []
        self.downsample_factor = 2
        self.tf = TransformListener()

        self.P = {}
        self.D = {}
        self.K = {}
        rospy.Subscriber("/color_camera/camera_info",
                         CameraInfo,
                         self.process_camera_info,
                         "color_camera",
                         queue_size=10)
        rospy.Subscriber("/fisheye_undistorted/camera_info",
                         CameraInfo,
                         self.process_camera_info,
                         "fisheye_undistorted",
                         queue_size=10)
        rospy.Subscriber("/point_cloud",
                         PointCloud,
                         self.process_point_cloud,
                         queue_size=10)
        rospy.Subscriber("/color_camera/image_raw/compressed",
                         CompressedImage,
                         self.process_image,
                         queue_size=10)
        rospy.Subscriber("/fisheye_undistorted/image_raw/compressed",
                         CompressedImage,
                         self.process_fisheye_image,
                         queue_size=10)
        self.transformed_pc_pub = rospy.Publisher('/transformed_pc', PointCloud, queue_size=10)
        self.back_transformed_pc_pub = rospy.Publisher('/back_transformed_pc', PointCloud, queue_size=10)

        self.clicked_point_pub = rospy.Publisher("/clicked_point", PointStamped, queue_size=10)
        self.color_camera_info = None
        self.depth_image = None
        self.points_3d = None
        self.image = None
        self.fisheye_image = None

        self.last_image_timestamp = None
        self.last_fisheye_image_timestamp = None

        self.depth_timestamp = None
        cv2.namedWindow("combined_feed")
        cv2.namedWindow("fisheye_feed")

    def process_fisheye_image(self,msg):
        # TODO: we are recompressing this later, might as well save the trouble and keep the compressed version around
        self.fisheye_image_list_lock.acquire()
        np_arr = np.fromstring(msg.data, np.uint8)
        self.last_fisheye_image_timestamp = msg.header.stamp
        self.fisheye_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        if len(self.fisheye_image_list) == self.image_list_max_size:
            self.fisheye_image_list.pop(0)
        self.fisheye_image_list.append((msg.header.stamp, self.fisheye_image))
        self.fisheye_image_list_lock.release()

    def process_image(self,msg):
        # TODO: we are recompressing this later, might as well save the trouble and keep the compressed version around
        self.image_list_lock.acquire()
        np_arr = np.fromstring(msg.data, np.uint8)
        self.last_image_timestamp = msg.header.stamp
        self.image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        if len(self.image_list) == self.image_list_max_size:
            self.image_list.pop(0)
        self.image_list.append((msg.header.stamp,self.image))
        self.image_list_lock.release()

    def process_camera_info(self, msg, camera_name):
        if camera_name == 'color_camera':
            self.color_camera_info = msg
        self.P[camera_name] = np.array(msg.P).reshape((3,4))
        self.K[camera_name] = np.array(msg.K).reshape((3,3))
        # TODO: this is necessary due to a mistake in intrinsics_server.py
        self.D[camera_name] = np.array([msg.D[0],msg.D[1],0,0,msg.D[2]])

    def get_nearest_image_temporally(self,timestamp):
        self.fisheye_image_list_lock.acquire()
        diff_list_fisheye = []
        for im_stamp,image in self.fisheye_image_list:
            diff_list_fisheye.append((abs((im_stamp-timestamp).to_sec()),image, im_stamp))
        closest_temporally_fisheye = min(diff_list_fisheye, key=lambda t: t[0])
        self.fisheye_image_list_lock.release()

        return closest_temporally_fisheye[1], closest_temporally_fisheye[2]


    def process_point_cloud(self, msg):
        self.depth_image_lock.acquire()
        try:
            self.depth_image_timestamp = msg.header.stamp
            self.points_3d = np.zeros((3,len(msg.points))).astype(np.float32)
            self.point_cloud_frame = msg.header.frame_id

            for i,p in enumerate(msg.points):
                # this is weird due to mismatches between OpenCV's camera coordinate system and ROS coordinate system conventions
                #self.points_3d[:,i] = np.array([-p.y, -p.z, p.x])
                self.points_3d[:,i] = np.array([p.x, p.y, p.z])
            self.depth_image_lock.release()
        except Exception as ex:
            print "Encountered an errror! ", ex
            self.depth_image_lock.release()

    @staticmethod
    def convert_pose_to_xy_and_theta(pose):
        orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        angles = euler_from_quaternion(orientation_tuple)
        return (pose.position.x, pose.position.y, angles[2])

    def run(self):
        r = rospy.Rate(10)
        while not(rospy.is_shutdown()):
            cv2.waitKey(5)

            if self.fisheye_image != None:
                cv2.imshow("fisheye_feed", self.fisheye_image)

            if self.fisheye_image != None and self.points_3d != None:
                self.depth_image_lock.acquire()
                nearest_fisheye_image, nearest_fisheye_image_timestamp = self.get_nearest_image_temporally(self.depth_image_timestamp)
                try:
                    self.tf.waitForTransform("device", "odom", nearest_fisheye_image_timestamp, rospy.Duration(1.0))
                    pose_msg = PoseStamped(header=Header(stamp=nearest_fisheye_image_timestamp,
                                                         frame_id="device"),
                                           pose=Pose())
                    transformed_pose = self.tf.transformPose('odom', pose_msg)
                    (x, y, theta) = DepthImageCreator.convert_pose_to_xy_and_theta(transformed_pose.pose)
                    retval, compressed_fisheye_img = cv2.imencode('.jpg', nearest_fisheye_image)

                    # store the point cloud in the odom frame as well
                    self.tf.waitForTransform("depth_camera", "odom", self.depth_image_timestamp, rospy.Duration(1.0))
                    transform_matrix = self.tf.asMatrix("odom",
                                                        Header(stamp=self.depth_image_timestamp,
                                                               frame_id="depth_camera"))
                    inv_transform_matrix = self.tf.asMatrix("depth_camera",
                                                            Header(stamp=self.depth_image_timestamp,
                                                                   frame_id="odom"))
                    # use the fisheye timestamp
                    fisheye_transform_matrix =  self.tf.asMatrix("odom",
                                                                 Header(stamp=nearest_fisheye_image_timestamp,
                                                                        frame_id="fisheye_camera"))
                    fisheye_inv_transform_matrix = self.tf.asMatrix("fisheye_camera",
                                                                    Header(stamp=nearest_fisheye_image_timestamp,
                                                                           frame_id="odom"))
                    if self.point_cloud_frame == 'depth_camera':
                        # undo the transform that we did right before calling projectPoints
                        odom_points = transform_matrix.dot(np.vstack((self.points_3d[0,:],
                                                                      self.points_3d[1,:],
                                                                      self.points_3d[2,:],
                                                                      np.ones((1,self.points_3d.shape[1])))))
                    else:
                        # undo transform, but otherwise leave things as before
                        odom_points = np.eye(4).dot(np.vstack((self.points_3d[0,:],
                                                               self.points_3d[1,:],
                                                               self.points_3d[2,:],
                                                               np.ones((1,self.points_3d.shape[1])))))
                    fisheye_camera_point_cloud = fisheye_inv_transform_matrix.dot(odom_points)

                    saved_combo = {'timestamp': self.depth_image_timestamp, 'x': x, 'y': y, 'theta': theta, 'fisheye_undistorted': compressed_fisheye_img, 'odom_points': odom_points.T, 'fisheye_camera_points': fisheye_camera_point_cloud.T, 'odom_to_depth_camera': inv_transform_matrix, 'depth_camera_to_odom': transform_matrix, 'odom_to_fisheye_camera': fisheye_inv_transform_matrix, 'fisheye_camera_to_odom': fisheye_transform_matrix, 'D': self.D, 'K': self.K}
                    self.combined_data.append(saved_combo)
                    print "added a new combo", len(self.combined_data)
                except Exception as ex:
                    print "Couldn't get the current device pose ", ex
                self.points_3d = None
                self.image = None
                self.depth_image_lock.release()
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('make_depth_image')
    node = DepthImageCreator()
    node.run()

    f = open('combo_sets.pickle', 'wt')
    pickle.dump(node.combined_data, f)
    f.close()