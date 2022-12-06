#!/usr/bin/env python
import rospy
from iq_gnc.py_gnc_functions import *
from datetime import datetime
import tf
import time
from geometry_msgs.msg import PointStamped
# from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node("droneController", anonymous=True)

    tfListener = tf.TransformListener()

    x = rospy.get_param("~x")
    y = rospy.get_param("~y")
    z = rospy.get_param("~z")
    id = rospy.get_param("~id")

    x = float(x)
    y = float(y)
    z = float(z)
    id = int(id)
    drone = gnc_api()
    drone.wait4connect()
    # drone.wait4start()
    drone.set_mode("GUIDED")
    drone.initialize_local_frame()
    drone.takeoff(1)

    ctime = datetime.now()
    rate = rospy.Rate(3)

    while abs((datetime.now()-ctime).total_seconds()<10):
        continue

    pt = PointStamped()
    pt.header.stamp = rospy.Time()
    pt.header.frame_id = 'world'
    # pt.point.x = -y
    # pt.point.y = x
    # pt.point.z = z

    pt.point.x = x
    pt.point.y = y
    pt.point.z = z

    pt = tfListener.transformPoint('iris{id}_odom'.format(id=id), pt)
    
    drone.set_destination(x=-pt.point.y, y=pt.point.x, z=pt.point.z, psi=0)
    while not drone.check_waypoint_reached():
        rate.sleep()
    
    print("Reached")

if __name__ == "__main__":
    main()