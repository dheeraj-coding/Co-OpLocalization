#!/usr/bin/env python
import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Pose, Point, Quaternion, PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from coop_localization.srv import PositionInfo
import numpy as np
import tf
import cv2
import torch
from torch.autograd.functional import jacobian

MARKER_LENGTH = 0.18
POSITION_SERVICE = '/iris{id}/position/position_info'

def rmse(xk, xj):
    return torch.sqrt(torch.mean((xk-xj)**2))

class Corrector:
    def __init__(self, odomk):
        self.Xk = torch.tensor([odomk.pose.pose.position.x, odomk.pose.pose.position.y, odomk.pose.pose.position.z], dtype=torch.float64)
        self.Xk = torch.reshape(self.Xk, (3, 1))

        self.covk = torch.tensor(odomk.pose.covariance, dtype=torch.float64)
        self.covk = torch.reshape(self.covk, (6, 6))
        self.covk = self.covk[:3, :3]

    def computeKalmanGainAndH(self, Xj, covj):
        res = jacobian(rmse, (self.Xk, Xj))
        Hk = torch.reshape(res[0], (1, 3))
        Hkt = torch.transpose(Hk, 0, 1)
        Hj = torch.reshape(res[1], (1, 3))
        Hjt = torch.transpose(Hj, 0, 1)
        
        term1 = torch.mm(Hk, self.covk)
        term1 = torch.mm(term1, Hkt)
        term2 = torch.mm(Hj, covj)
        term2 = torch.mm(term2, Hjt) 
        Skj = term1 + term2

        gainkj = torch.mm(self.covk, Hkt)
        gainkj = torch.mm(gainkj, torch.linalg.pinv(Skj))
        return gainkj, Hk

    def correctPositionAndCovariance(self, odomj, estimatedDist):
        Xj = torch.tensor([odomj.pose.pose.position.x, odomj.pose.pose.position.y, odomj.pose.pose.position.z], dtype=torch.float64)
        Xj = torch.reshape(Xj, (3, 1))
        covj = torch.tensor(odomj.pose.covariance, dtype=torch.float64)
        covj = torch.reshape(covj, (6, 6))
        covj = covj[:3, :3]
        estimatedDist = torch.tensor(estimatedDist, dtype=torch.float64)
        estimatedDist = torch.reshape(estimatedDist, (1, 1))

        gainkj, Hk = self.computeKalmanGainAndH(Xj, covj)
        self.correctPosition(Xj, estimatedDist, gainkj)
        self.correctCovariance(gainkj, Hk)
    
    def correctPosition(self, Xj, estimatedDist, gainkj):
        zkj = rmse(self.Xk, Xj)
        zkj = torch.reshape(zkj, (1, 1))
        term2 = torch.mm(gainkj, estimatedDist - zkj)
        self.Xk = self.Xk + term2
    
    def correctCovariance(self, gainkj, Hk):
        term2 = torch.mm(gainkj, Hk)
        shape = term2.size()
        I = torch.eye(shape[0], shape[1])
        multiplier = I-term2
        self.covk = torch.mm(multiplier, self.covk)
    def getPointStamped(self):
        pt = PointStamped()
        pt.point.x = self.Xk[0].item()
        pt.point.y = self.Xk[1].item()
        pt.point.z = self.Xk[2].item()
        return pt

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

        ts = message_filters.ApproximateTimeSynchronizer([image_sub, cam_info, odom_sub], 5, 0.6)
        ts.registerCallback(self.handleSynchronizedCallback)
        self.stamp = None

        self.ptPublisher = rospy.Publisher('~qrpoint', PointStamped, queue_size=20)

        rospy.spin()
    
    def getDroneID(self, id):
        return id // 5
    
    def getBoxID(self, id):
        return id % 5
    
    def computeDist(self, tVec):
        distance = np.sqrt(tVec[0][0]**2+tVec[0][1]**2+tVec[0][2]**2)
        return distance

    
    def drawAxes(self, id, img, corner, tVec, rVec, intrinsic, distortion):
        cv2.polylines(img, [corner.astype(np.int32)], True, (0, 255, 255), 1, cv2.LINE_AA)
        corner = corner.reshape(4, 2)
        corner = corner.astype(int)
        top_right = corner[0].ravel()
        bottom_right = corner[2].ravel()
        distance = self.computeDist(tVec)
        _ = cv2.drawFrameAxes(img, intrinsic, distortion, rVec, tVec, 1, 1)
        img = cv2.putText(img, "id: {id} Dist: {dist: .2f}".format(id=id, dist=distance), tuple(top_right), cv2.FONT_HERSHEY_PLAIN, 1.3, (0, 0, 255), 2, cv2.LINE_AA)
        img = cv2.putText(img, f"x:{round(tVec[0][0], 1)} y:{round(tVec[0][1], 1)} z:{round(tVec[0][2], 1)}", tuple(bottom_right), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255), 2, cv2.LINE_AA)

        return img
    
    def transformToMapFrame(self, id, stamp, tVec):
        droneId = self.getDroneID(id)
        boxId = self.getBoxID(id)
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
    
    def transformOdom(self, odom):
        pose = PoseStamped()
        pose.pose = odom.pose.pose
        pose.header.frame_id = 'iris{id}_odom'.format(id=self.id)
        pose.header.stamp = self.stamp
        transformedPose = None
        try:
            transformedPose = self.tfListener.transformPose("map", pose)
            return transformedPose.pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("error transforming odom %s" % e)
        return odom.pose.pose
    
    def obtainPositionFromService(self, id, tstamp):
        srv = POSITION_SERVICE.format(id=id)
        rospy.wait_for_service(srv)
        try:
            positionRequest = rospy.ServiceProxy(srv, PositionInfo)
            resp = positionRequest(tstamp)
            return resp
        except rospy.ServiceException as e:
            print("Service call to iris%d failed: %s" % (id, e))

    def processImage(self, img, camInfo, corrector: Corrector, drawAxes=False):
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
                if len(tVec) < 3:
                    continue
                if drawAxes:
                    self.drawAxes(id, img, corner, tVec, rVec, camMat, camInfo.D)
                # print(self.obtainPositionFromService(self.getDroneID(id), camInfo.header.stamp))
                resp = self.obtainPositionFromService(self.getDroneID(id), camInfo.header.stamp)
                if resp and resp.stationary:
                    tVec[0] = -tVec[0]
                    tVec[1] = -tVec[1]
                    corrector.correctPositionAndCovariance(resp.pos, self.computeDist(tVec))
                continue
        return img

    def handleSynchronizedCallback(self, img, camInfo, odom):
        self.stamp = rospy.Time(0)

        try:
            cv2img = self.bridge.imgmsg_to_cv2(img, 'bgr8')
            odom.pose.pose = self.transformOdom(odom)
            corrector = Corrector(odom)
            processed = self.processImage(cv2img, camInfo, corrector, drawAxes=False)
            pt = corrector.getPointStamped()
            pt.header.frame_id = 'map'
            pt.header.stamp = camInfo.header.stamp
            print(pt)
            self.ptPublisher.publish(pt)

            # cv2.imshow('droneImg', processed)
            # cv2.waitKey(100)
            # cv2.destroyAllWindows()
        except CvBridgeError as e:
            print(e)


if __name__ == "__main__":
    fuser = EKFfusion()
    # odom = Odometry()
    # odom.pose.pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
    # odom.pose.covariance = [0.0]*36

    # odom2 = Odometry()
    # odom2.pose.pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
    # odom2.pose.covariance = [0.0]*36

    # corr = Corrector(odom)
    # corr.correctPositionAndCovariance(odom2, 2.3)
