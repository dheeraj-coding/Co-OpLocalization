#!/usr/bin/env python
import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tf
import cv2

MARKER_LENGTH = 0.18

class EKFfusion:
    def __init__(self):
        rospy.init_node('ekf_fuser', anonymous=True)
        self.bridge = CvBridge()
        id = rospy.get_param("~id")
        self.id = int(id)
        
        image_sub = message_filters.Subscriber('/iris{id}/image_raw'.format(id=id), Image)
        cam_info = message_filters.Subscriber('/iris{id}/camera_info'.format(id=id), CameraInfo)
        odom_sub = message_filters.Subscriber('/iris{id}/mavros/local_position/odom'.format(id=id), Odometry)

        self.tfListener = tf.TransformListener()

        ts = message_filters.ApproximateTimeSynchronizer([image_sub, cam_info, odom_sub], 10, 0.5)
        ts.registerCallback(self.handleSynchronizedCallback)
        self.stamp = None
        rospy.spin()
    
    def drawAxes(self, id, img, corner, tVec, rVec, intrinsic, distortion):
        cv2.polylines(img, [corner.astype(np.int32)], True, (0, 255, 255), 1, cv2.LINE_AA)
        corner = corner.reshape(4, 2)
        corner = corner.astype(int)
        top_right = corner[0].ravel()
        bottom_right = corner[2].ravel()
        distance = np.sqrt(tVec[0][0]**2+tVec[0][1]**2+tVec[0][2]**2)
        _ = cv2.drawFrameAxes(img, intrinsic, distortion, rVec, tVec, 1, 1)
        img = cv2.putText(img, "id: {id} Dist: {dist: .2f}".format(id=id, dist=distance), tuple(top_right), cv2.FONT_HERSHEY_PLAIN, 1.3, (0, 0, 255), 2, cv2.LINE_AA)
        img = cv2.putText(img, f"x:{round(tVec[0][0], 1)} y:{round(tVec[0][1], 1)} z:{round(tVec[0][2], 1)}", tuple(bottom_right), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255), 2, cv2.LINE_AA)

        return img
    
    def transformToMapFrame(self, id, stamp, tVec):
        droneId = id // 5
        boxId = id % 5
        pt = PointStamped()
        pt.header.stamp = stamp
        if boxId == 0:
            pt.header.frame_id = "iris{id}_front".format(id=droneId)
        elif boxId == 1:
            pt.header.frame_id = "iris{id}_right".format(id=droneId)
        elif boxId == 2:
            pt.header.frame_id = "iris{id}_left".format(id=droneId)
        elif boxId == 3:
            pt.header.frame_id = "iris{id}_back".format(id=droneId)
        elif boxId == 4:
            pt.header.frame_id = "iris{id}_top".format(id=droneId)
        
        pt.point.x = tVec[0][0]
        pt.point.y = tVec[0][1]
        pt.point.z = tVec[0][2]
        transformedPt = None
        try:
            transformedPt = self.tfListener.transformPoint("map", pt)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("error transforming point %s"%e)
        return transformedPt
    
    def processImage(self, img, camInfo):
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        arucoParams = cv2.aruco.DetectorParameters_create()
        grayframe = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        (corners, ids, _) = cv2.aruco.detectMarkers(grayframe, arucoDict, parameters=arucoParams)
        if corners:
            camMat = np.asarray(camInfo.K)
            camMat = camMat.reshape(3, 3)
            rVecList, tVecList, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_LENGTH, camMat, camInfo.D)
            totalMarkers = range(0, ids.size)
            for id, corner, i in zip(ids, corners, totalMarkers):
                id = id[0]
                tVec = tVecList[i]
                rVec = rVecList[i]
                self.drawAxes(id, img, corner, tVec, rVec, camMat, camInfo.D)
                print(self.transformToMapFrame(id, self.stamp, tVec))
                continue
        return img

    def handleSynchronizedCallback(self, img, camInfo, odom):
        self.stamp = rospy.Time(0)

        try:
            cv2img = self.bridge.imgmsg_to_cv2(img, 'bgr8')
            processed = self.processImage(cv2img, camInfo)
            cv2.imshow('droneImg', processed)
            cv2.waitKey(100)
            cv2.destroyAllWindows()
        except CvBridgeError as e:
            print(e)


if __name__ == "__main__":
    fuser = EKFfusion()
