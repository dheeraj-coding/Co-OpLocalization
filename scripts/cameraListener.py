#!/usr/bin/env python
from symbol import parameters
import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
import numpy as np
import tf
from geometry_msgs.msg import PointStamped, Point

import cv2

bridge = CvBridge()
MARKER_LENGTH = 0.18
tfListener = None
ptPublisher = None
myid = None

def transformWaiter(fromFrame, toFrame):
    print("Wwaiting on ", fromFrame, "to ", toFrame)
    global tfListener
    tfListener.waitForTransform(fromFrame, toFrame, rospy.Time(), rospy.Duration(4.0))
    while not rospy.is_shutdown():
        try:
            tfListener.waitForTransform(fromFrame, toFrame, rospy.Time().now(), rospy.Duration(4.0))
            break
        except e:
            print("TF wait error: ")


def callback(image, camera_info):
    # rospy.loginfo(rospy.get_caller_id()+"I heard %s", data.data)
    print("Received Image")
    global MARKER_LENGTH
    global tfListener
    try:
        cv2_img = bridge.imgmsg_to_cv2(image, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        # decoded_data = decode(cv2_img)
        # print(decoded_data)
        # for d in decoded_data:
            # print(d.polygon)
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        arucoParams = cv2.aruco.DetectorParameters_create()
        gray_frame = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
        (corners, ids, rejected) = cv2.aruco.detectMarkers(gray_frame, arucoDict, parameters=arucoParams)
        
        if corners:
            cam_mat = np.asarray(camera_info.K)
            cam_mat = cam_mat.reshape(3, 3)
            rVec, tVec, _= cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_LENGTH, cam_mat, camera_info.D)
            total_markers = range(0, ids.size)
            for id, corn, i in zip(ids, corners, total_markers):
                cv2.polylines(cv2_img, [corn.astype(np.int32)], True, (0, 255, 255), 1, cv2.LINE_AA)
                corn = corn.reshape(4, 2)
                corn = corn.astype(int)
                top_right = corn[0].ravel()
                print(type(top_right))
                top_left = corn[1].ravel()
                bottom_right = corn[2].ravel()
                bottom_left = corn[3].ravel()

                distance = np.sqrt(tVec[i][0][2]**2+tVec[i][0][0]**2+tVec[i][0][1]**2)
                point = cv2.drawFrameAxes(cv2_img, cam_mat, camera_info.D, rVec[i], tVec[i], 1, 1)
                cv2_img = cv2.putText(cv2_img, "id: {id} Dist: {dist: .2f}".format(id=id[0], dist=distance), tuple(top_right), cv2.FONT_HERSHEY_PLAIN, 1.3, (0, 0, 255), 2, cv2.LINE_AA)
                cv2_img = cv2.putText(cv2_img, f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} z: {round(tVec[i][0][2], 1)}", tuple(bottom_right), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255), 2,cv2.LINE_AA)
                
                id = id[0]
                drone_id = id // 5
                box_id = id % 5
                pt = PointStamped()
                pt.header.stamp = rospy.Time(0).now()
                if box_id == 0:
                    pt.header.frame_id = "iris"+str(drone_id)+"_front"
                elif box_id == 1:
                    pt.header.frame_id = "iris"+str(drone_id)+"_right"
                elif box_id == 2:
                    pt.header.frame_id = "iris"+str(drone_id)+"_left"
                elif box_id == 3:
                    pt.header.frame_id = "iris"+str(drone_id)+"_back"
                elif box_id == 4:
                    pt.header.frame_id = "iris"+str(drone_id)+"_top"

                pt.point.x = tVec[i][0][1]
                pt.point.y = tVec[i][0][0]  
                pt.point.z = tVec[i][0][2]

                try:
                    transformWaiter(pt.header.frame_id, "world")
                    resP = tfListener.transformPoint("world", pt)
                    # print(resP)
                    ptPublisher.publish(resP)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    print("Transform error")
                    print(e)
                    continue



        # cv2.imshow('droneImg', cv2_img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

def listener():
    global ptPublisher
    global tfListener
    global myid 
    rospy.init_node('listener', anonymous=True)
    # rospy.Subscriber('chatter', String, callback)
    # rospy.spin()

    id = rospy.get_param("~id")
    id = int(id)
    print("ID= ", id)
    myid = id

    tfListener = tf.TransformListener()

    ptPublisher = rospy.Publisher('~pointInfo', PointStamped, queue_size=100)

    image_sub = message_filters.Subscriber('/iris{id}/image_raw'.format(id=id), Image)
    info_sub = message_filters.Subscriber('/iris{id}/camera_info'.format(id=id), CameraInfo)


    ts = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == "__main__":
    listener()