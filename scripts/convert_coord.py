#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import *
import numpy as np

class Converter:
    def __init__(self):
        rospy.init_node("converter", anonymous=True)
        id = rospy.get_param("~id")
        id = int(id)
        sub = rospy.Subscriber('/iris{id}/mavros/local_position/odom'.format(id=id), Odometry, self.callback)
        self.posepub = rospy.Publisher('~pose', PoseStamped)
        self.pub = rospy.Publisher('~odom', Odometry)
        rospy.spin()
    def callback(self, msg):
        print("converting...")
        # msg.pose.pose.position.y = -msg.pose.pose.position.y
        t = msg.pose.pose.position.x
        msg.pose.pose.position.x = msg.pose.pose.position.y
        msg.pose.pose.position.y = -t
        rpy = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        rpy = np.asarray(rpy)
        rpy[1] = -rpy[1]
        quat = quaternion_from_euler(rpy[0],rpy[1],rpy[2])

        q_rot = quaternion_from_euler(0, 0, -math.pi/2.0)
        q_new = quaternion_multiply(q_rot, quat)

        msg.pose.pose.orientation.x = q_new[0]
        msg.pose.pose.orientation.y = q_new[1]
        msg.pose.pose.orientation.z = q_new[2]
        msg.pose.pose.orientation.w = q_new[3]
        msg.twist.twist.linear.y = -msg.twist.twist.linear.y 
        msg.twist.twist.angular.y = -msg.twist.twist.angular.y
        self.pub.publish(msg) 

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.posepub.publish(pose)

def main():
    c = Converter()

if __name__ == "__main__":
    main()