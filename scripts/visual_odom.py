#!/usr/bin/env python
import os
import numpy as np
import cv2
import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from cv_bridge import CvBridge, CvBridgeError
import queue
import threading
from tf.transformations import quaternion_matrix, quaternion_from_matrix
import plotting


from tqdm import tqdm

class VisualOdometry():
    def __init__(self, data_dir):
        # self.K, self.P = self._load_calib(os.path.join(data_dir, 'calib.txt'))
        # self.gt_poses = self._load_poses(os.path.join(data_dir,"poses.txt"))
        # self.images = self._load_images(os.path.join(data_dir,"image_l"))
        self.orb = cv2.ORB_create(3000)
        FLANN_INDEX_LSH = 6
        index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(indexParams=index_params, searchParams=search_params)

        self.bridge = CvBridge()

        rospy.init_node('visual_odom', anonymous=True)
        img_topic = rospy.get_param("~image")
        info_topic = rospy.get_param("~info")
        pose_topic = rospy.get_param("~pose")

        image_sub = message_filters.Subscriber(img_topic, Image)
        info_sub = message_filters.Subscriber(info_topic, CameraInfo)

        ts = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
        ts.registerCallback(self.callback)

        self.q = queue.Queue(1)
        self.prevImg = None
        self.prevInfo = None

        self.pub = rospy.Publisher('visodom/pose', PoseStamped, queue_size=100)

        self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.poseInitializer)

        self.init_pose = None
    
    def poseInitializer(self, p_stamp):
        homo = quaternion_matrix([p_stamp.pose.orientation.x, p_stamp.pose.orientation.y, p_stamp.pose.orientation.z, p_stamp.pose.orientation.w])
        homo[0, 3] = p_stamp.pose.position.x
        homo[1, 3] = p_stamp.pose.position.y
        homo[2, 3] = p_stamp.pose.position.z
        self.init_pose = homo
        self.pose_sub.unregister()


    
    def launch(self):
        proc_thread = threading.Thread(target=self.processor)
        proc_thread.start()
        rospy.spin()
        proc_thread.join()
    
    def processor(self):
        print("Starting..... Processor")
        
        while self.init_pose is None:
            continue

        cur_pose = self.init_pose

        self.prevImg, self.prevInfo = self.q.get()
        self.prevImg = self.bridge.imgmsg_to_cv2(self.prevImg, 'bgr8')
        self.P = np.asarray(self.prevInfo.P)
        self.K = np.asarray(self.prevInfo.K)
        self.P = self.P.reshape((3, 4))
        self.K = self.K.reshape((3,3))
        # cur_pose = np.asarray([
        #     [1.000000e+00, 1.197625e-11, 1.704638e-10, 0.000000e+00],
        #     [1.197625e-11, 1.000000e+00, 3.562503e-10, -1.110223e-16],
        #     [1.704638e-10, 3.562503e-10, 1.000000e+00, 2.220446e-16],
        # ])
        
        # np.vstack([cur_pose, [0, 0, 0, 1]])


        while True:
            item = self.q.get()
            if item is None:
                break
            img, info = item
            cvimg = self.bridge.imgmsg_to_cv2(img, 'bgr8')

            q1, q2 = self.get_matches(self.prevImg, cvimg)
            transf = self.get_pose(q1, q2)
            transf = np.nan_to_num(transf, neginf=0,posinf=0)
            # print(transf)
            cur_pose = np.matmul(cur_pose, np.linalg.inv(transf))
            cur_pose = np.vstack([cur_pose, [0, 0, 0, 1]])
            

            q = quaternion_from_matrix(cur_pose)
            
            ros_pose = PoseStamped()
            ros_pose.pose.position.x = cur_pose[0, 3]
            ros_pose.pose.position.y = cur_pose[2, 3]
            ros_pose.pose.position.z = cur_pose[1, 3]
            ros_pose.pose.orientation.x = q[0]
            ros_pose.pose.orientation.y = q[1]
            ros_pose.pose.orientation.z = q[2]
            ros_pose.pose.orientation.w = q[3]

            ros_pose.header = info.header
            ros_pose.header.frame_id = 'map'

            print(ros_pose)

            self.pub.publish(ros_pose)

            self.prevImg = cvimg
            self.prevInfo = info

            # cv2.imshow("Image received", item)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
        print("Exiting.... Processor")
    
    def callback(self, image, camera_info):
        try:
            # cv2_img = self.bridge.imgmsg_to_cv2(image, 'bgr8')
            self.q.put((image, camera_info), block=False)
        except (CvBridgeError, queue.Full) as e:
            print(e)

    @staticmethod
    def _load_calib(filepath):
        """
        Loads the calibration of the camera
        Parameters
        ----------
        filepath (str): The file path to the camera file

        Returns
        -------
        K (ndarray): Intrinsic parameters
        P (ndarray): Projection matrix
        """
        with open(filepath, 'r') as f:
            params = np.fromstring(f.readline(), dtype=np.float64, sep=' ')
            P = np.reshape(params, (3, 4))
            K = P[0:3, 0:3]
        return K, P

    @staticmethod
    def _load_poses(filepath):
        """
        Loads the GT poses

        Parameters
        ----------
        filepath (str): The file path to the poses file

        Returns
        -------
        poses (ndarray): The GT poses
        """
        poses = []
        with open(filepath, 'r') as f:
            for line in f.readlines():
                T = np.fromstring(line, dtype=np.float64, sep=' ')
                T = T.reshape(3, 4)
                T = np.vstack((T, [0, 0, 0, 1]))
                poses.append(T)
        return poses

    @staticmethod
    def _load_images(filepath):
        """
        Loads the images

        Parameters
        ----------
        filepath (str): The file path to image dir

        Returns
        -------
        images (list): grayscale images
        """
        image_paths = [os.path.join(filepath, file) for file in sorted(os.listdir(filepath))]
        return [cv2.imread(path, cv2.IMREAD_GRAYSCALE) for path in image_paths]

    @staticmethod
    def _form_transf(R, t):
        """
        Makes a transformation matrix from the given rotation matrix and translation vector

        Parameters
        ----------
        R (ndarray): The rotation matrix
        t (list): The translation vector

        Returns
        -------
        T (ndarray): The transformation matrix
        """
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = t
        return T

    def get_matches(self, prev, cur):
        """
        This function detect and compute keypoints and descriptors from the i-1'th and i'th image using the class orb object

        Parameters
        ----------
        i (int): The current frame

        Returns
        -------
        q1 (ndarray): The good keypoints matches position in i-1'th image
        q2 (ndarray): The good keypoints matches position in i'th image
        """
        # Find the keypoints and descriptors with ORB
        # kp1, des1 = self.orb.detectAndCompute(self.images[i - 1], None)
        kp1, des1 = self.orb.detectAndCompute(prev, None)
        # kp2, des2 = self.orb.detectAndCompute(self.images[i], None)
        kp2, des2 = self.orb.detectAndCompute(cur, None)
        # Find matches
        matches = self.flann.knnMatch(des1, des2, k=2)

        # Find the matches there do not have a to high distance
        good = []
        try:
            for m, n in matches:
                if m.distance < 0.8 * n.distance:
                    good.append(m)
        except ValueError:
            pass

        draw_params = dict(matchColor = -1, # draw matches in green color
                 singlePointColor = None,
                 matchesMask = None, # draw only inliers
                 flags = 2)

        # img3 = cv2.drawMatches(self.images[i], kp1, self.images[i-1],kp2, good ,None,**draw_params)
        # cv2.imshow("image", img3)
        # cv2.waitKey(200)

        # Get the image points form the good matches
        q1 = np.float32([kp1[m.queryIdx].pt for m in good])
        q2 = np.float32([kp2[m.trainIdx].pt for m in good])
        return q1, q2

    def get_pose(self, q1, q2):
        """
        Calculates the transformation matrix

        Parameters
        ----------
        q1 (ndarray): The good keypoints matches position in i-1'th image
        q2 (ndarray): The good keypoints matches position in i'th image

        Returns
        -------
        transformation_matrix (ndarray): The transformation matrix
        """
        # Essential matrix
        E, _ = cv2.findEssentialMat(q1, q2, self.K, threshold=1)

        # Decompose the Essential matrix into R and t
        R, t = self.decomp_essential_mat(E, q1, q2)

        # Get transformation matrix
        transformation_matrix = self._form_transf(R, np.squeeze(t))
        return transformation_matrix

    def decomp_essential_mat(self, E, q1, q2):
        """
        Decompose the Essential matrix

        Parameters
        ----------
        E (ndarray): Essential matrix
        q1 (ndarray): The good keypoints matches position in i-1'th image
        q2 (ndarray): The good keypoints matches position in i'th image

        Returns
        -------
        right_pair (list): Contains the rotation matrix and translation vector
        """
        def sum_z_cal_relative_scale(R, t):
            # Get the transformation matrix
            T = self._form_transf(R, t)
            # Make the projection matrix
            P = np.matmul(np.concatenate((self.K, np.zeros((3, 1))), axis=1), T)

            # Triangulate the 3D points
            hom_Q1 = cv2.triangulatePoints(self.P, P, q1.T, q2.T)
            # Also seen from cam 2
            hom_Q2 = np.matmul(T, hom_Q1)

            # Un-homogenize
            uhom_Q1 = hom_Q1[:3, :] / hom_Q1[3, :]
            uhom_Q2 = hom_Q2[:3, :] / hom_Q2[3, :]

            # Find the number of points there has positive z coordinate in both cameras
            sum_of_pos_z_Q1 = sum(uhom_Q1[2, :] > 0)
            sum_of_pos_z_Q2 = sum(uhom_Q2[2, :] > 0)

            # Form point pairs and calculate the relative scale
            relative_scale = np.mean(np.linalg.norm(uhom_Q1.T[:-1] - uhom_Q1.T[1:], axis=-1)/
                                     np.linalg.norm(uhom_Q2.T[:-1] - uhom_Q2.T[1:], axis=-1))
            return sum_of_pos_z_Q1 + sum_of_pos_z_Q2, relative_scale

        # Decompose the essential matrix
        R1, R2, t = cv2.decomposeEssentialMat(E)
        t = np.squeeze(t)

        # Make a list of the different possible pairs
        pairs = [[R1, t], [R1, -t], [R2, t], [R2, -t]]

        # Check which solution there is the right one
        z_sums = []
        relative_scales = []
        for R, t in pairs:
            z_sum, scale = sum_z_cal_relative_scale(R, t)
            z_sums.append(z_sum)
            relative_scales.append(scale)

        # Select the pair there has the most points with positive z coordinate
        right_pair_idx = np.argmax(z_sums)
        right_pair = pairs[right_pair_idx]
        relative_scale = relative_scales[right_pair_idx]
        R1, t = right_pair
        t = t * relative_scale

        return [R1, t]


def main():
    data_dir = "KITTI_sequence_2"  # Try KITTI_sequence_2 too
    vo = VisualOdometry(data_dir)
    vo.launch()
    plotting.visualize_paths(vo.path, vo.path, "VisOdom", file_out="VisOdom.html")
    # play_trip(vo.images)  # Comment out to not play the trip

    # gt_path = []
    # estimated_path = []
    # for i, gt_pose in enumerate(tqdm(vo.gt_poses, unit="pose")):
    #     if i == 0:
    #         cur_pose = gt_pose
    #     else:
    #         q1, q2 = vo.get_matches(i)
    #         transf = vo.get_pose(q1, q2)
    #         cur_pose = np.matmul(cur_pose, np.linalg.inv(transf))
    #     gt_path.append((gt_pose[0, 3], gt_pose[2, 3]))
    #     estimated_path.append((cur_pose[0, 3], cur_pose[2, 3]))
    # plotting.visualize_paths(gt_path, estimated_path, "Visual Odometry", file_out=os.path.basename(data_dir) + ".html")


if __name__ == "__main__":
    main()
