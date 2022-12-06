#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, PoseWithCovariance, Quaternion
from tf.transformations import quaternion_from_matrix
import torch
from torch.autograd.functional import jacobian
import math

class IPose:
    def __init__(self):
        self.pos = torch.tensor([[0], [0], [0]], dtype=torch.float64)
        self.orien = torch.eye(3, dtype=torch.float64)
        self.covariance = torch.zeros((3,3), dtype=torch.float64)
    def toROSPoseWithCovariance(self):
        rpose = PoseWithCovariance()
        rpose.pose.position = Point(self.pos[0].item(), self.pos[1].item(), self.pos[2].item())
        # rpose.pose.orientation = quaternion_from_matrix(self.orien)
        rpose.pose.orientation = Quaternion(0, 0, 0, 1)
        cov = torch.zeros((6, 6))
        cov[:3, :3] = self.covariance
        cov = cov.reshape(36)
        rpose.covariance = cov.tolist()
        return rpose  

def computePos(pos, vel, deltaT):
    npos = pos + deltaT * vel
    return npos


class DeadReckoning:
    def __init__(self):
        rospy.init_node("dead_reckoning", anonymous=True)
        id = rospy.get_param("~id")
        self.id = int(id)

        imuSubscriber = rospy.Subscriber('/iris{id}/mavros/imu/data'.format(id=self.id), Imu, self.imuCallback)

        self.pose = IPose()
        self.time = None
        self.gravity = torch.tensor([[0], [0], [0]], dtype=torch.float64)
        self.velocity = torch.tensor([[0], [0], [0]], dtype=torch.float64)

        self.posePublisher = rospy.Publisher('dead_reckon/pose', PoseWithCovarianceStamped, queue_size=200)

        self.deltaT = None
        self.first = True
        self.seq = 0

        self.publishPose()

        rospy.spin()

    def setGravity(self, lacc):
        self.gravity[0] = lacc.x
        self.gravity[1] = lacc.y
        self.gravity[2] = lacc.z

    def calcPositionAndCovariance(self, lacc, lcov):
        acc_l = torch.tensor([[lacc.x], [lacc.y], [lacc.z]], dtype=torch.float64)
        acc_g = torch.mm(self.pose.orien, acc_l)
        self.velocity = self.velocity + self.deltaT * (acc_g - self.gravity)
        self.pose.pos = self.pose.pos + self.deltaT * self.velocity

        res = jacobian(computePos, (torch.reshape(self.pose.pos, (1, 3)), torch.reshape(self.velocity, (1, 3)), torch.tensor(self.deltaT)))
        G = res[0].reshape((3, 3))
        V = res[1].reshape((3, 3))
        controlCov = torch.tensor(lcov, dtype=torch.float64)
        controlCov = controlCov.reshape((3, 3))
        t1 = torch.mm(G, torch.mm(self.pose.covariance, torch.transpose(G, 0, 1)))
        t2 = torch.mm(V, torch.mm(controlCov, torch.transpose(V, 0, 1)))
        self.pose.covariance = t1 + t2

    def calcOrientation(self, lang):
        B = torch.tensor([
            [0, -lang.z * self.deltaT, lang.y * self.deltaT],
            [lang.z * self.deltaT, 0, -lang.x * self.deltaT],
            [-lang.y * self.deltaT, lang.x * self.deltaT, 0]
        ])
        sigma = math.sqrt(lang.x ** 2 + lang.y ** 2 + lang.z ** 2) * self.deltaT

        t1 = math.sin(sigma) * B
        t2 = (1-math.cos(sigma) / (sigma ** 2))
        t3 = t2 * torch.mm(B, B)
        t4 = torch.eye(3, dtype=torch.float64) + t1 + t3
        self.pose.orien = torch.mm(self.pose.orien, t4)


    def imuCallback(self, data:Imu):
        if(self.first):
            self.time = data.header.stamp
            self.deltaT = 0
            self.setGravity(data.linear_acceleration)
            self.first = False
        else:
            self.deltaT = (data.header.stamp - self.time).to_sec();
            self.time = data.header.stamp
            self.calcOrientation(data.angular_velocity)
            self.calcPositionAndCovariance(data.linear_acceleration, data.linear_acceleration_covariance)
            self.publishPose()
    
    def publishPose(self):
        p = PoseWithCovarianceStamped()
        p.header.frame_id = 'iris{id}_odom'.format(id=self.id)
        p.header.stamp = rospy.Time()
        p.header.seq = self.seq
        self.seq += 1
        # p.pose.pose = self.pose.toROSPose()
        # p.pose.covariance = self.pose.covariance
        p.pose = self.pose.toROSPoseWithCovariance()
        print(p)
        self.posePublisher.publish(p)  

if __name__ == "__main__":
    dr = DeadReckoning()