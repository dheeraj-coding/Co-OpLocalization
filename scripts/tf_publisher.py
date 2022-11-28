#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import time
import tf

class DroneTfPublisher:
    def __init__(self, id) -> None:
        self.id = id
        self.drone_name = 'iris{id}'.format(id=id)
    def handle_drone_pose(self, pose_msg):
        br = tf.TransformBroadcaster()
        br.sendTransform((pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z),
                    (pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w),
                    rospy.Time.now(),
                    self.drone_name,
                    self.drone_name+"_odom")

if __name__=="__main__":
    rospy.init_node("drone_tf_publisher", anonymous=True)
    num_drones = rospy.get_param("~num")
    for i in range(num_drones):
        dpub = DroneTfPublisher(i+1)
        rospy.Subscriber('/iris{id}/mavros/local_position/pose'.format(id=i+1), PoseStamped, dpub.handle_drone_pose)
    rospy.spin()