#!/usr/bin/env python
import rospy
# from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PointStamped
import numpy as np
import sys

class Collector:
    def __init__(self, id, suffix) -> None:
        self.id = id 
        self.suffix = suffix
        self.gt = np.asarray([[0, 0, 0]])
        self.est = np.asarray([[0, 0, 0]])
    def eventLoop(self):
        rospy.init_node("data_collector", anonymous=True)

        ptsub = rospy.Subscriber('/iris{id}/camestimator/pointInfo'.format(id=self.id), PointStamped, self.ptCallback)
        self.gtCoords = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        rospy.on_shutdown(self.onShutDown)

        rospy.spin()
    
    def onShutDown(self):
        print("Shutting down....")
        print(self.gt.shape)
        gtname = 'iris{id}_{suff}_gt.csv'.format(id=self.id, suff=self.suffix)
        estname = 'iris{id}_{suff}_est.csv'.format(id=self.id, suff=self.suffix)
        # self.gt.tofile(gtname, sep=',')
        # self.est.tofile(estname, sep=',')
        np.savetxt(gtname, self.gt, delimiter=",")
        np.savetxt(estname, self.est, delimiter=',')
            
    def ptCallback(self, pt:PointStamped):
        print("Called back")
        estPt = np.asarray([pt.point.x, pt.point.y, pt.point.z], dtype=np.float64)
        # self.est.append(estPt)
        self.est = np.vstack((self.est, estPt))
        modelCoords = self.gtCoords('iris{id}'.format(id=self.id), '')
        gtposition = modelCoords.pose.position
        gtPt = np.asarray([gtposition.x, gtposition.y, gtposition.z], dtype=np.float64)
        # self.gt.append(gtPt)
        self.gt = np.vstack((self.gt, gtPt))
        pass

if __name__ == "__main__":
    id = int(sys.argv[1])
    suffix = sys.argv[2]
    col = Collector(id, suffix)
    col.eventLoop()
