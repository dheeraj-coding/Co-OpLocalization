#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import message_filters
from coop_localization.srv import PositionInfo, PositionInfoResponse
from coop_localization.srv import Stagnant, StagnantResponse
import tf

class PositionService:
    def __init__(self) -> None:
        rospy.init_node('positionNode', anonymous=True)
        self.tfListener = tf.TransformListener()
        
        self.stagnant = True

        id = rospy.get_param("~id")
        self.id = int(id)
        myTopic = '/iris{id}/mavros/local_position/odom'.format(id=id)

        odomSub = message_filters.Subscriber(myTopic, Odometry)
        self.cache = message_filters.Cache(odomSub, 500)
    
        rospy.Service('~position_info', PositionInfo, self.handlePositionRequest)
        rospy.Service('~stagnant', Stagnant, self.handleStagnantRequest)
        rospy.spin()

    def handleStagnantRequest(self, status):
        self.stagnant = status.stagnant
        return StagnantResponse(True)
        
    
    def handlePositionRequest(self, req):
        odom = self.cache.getElemBeforeTime(req.stamp)
        if odom is not None:
            pose = PoseStamped()
            pose.header.frame_id = 'iris{id}_odom'.format(id=self.id)
            pose.header.stamp = odom.header.stamp
            pose.pose = odom.pose.pose
            pose = self.tfListener.transformPose("map", pose)
            odom.pose.pose = pose.pose
        stationary = self.stagnant
        return PositionInfoResponse(odom, stationary)
if __name__ == "__main__":
    srv = PositionService()