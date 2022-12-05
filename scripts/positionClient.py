#!/usr/bin/env python
import rospy
from coop_localization.srv import *
import time

POSITION_SERVICE = '/iris1/position/position_info'
STAGNANT_SERVICE = 'stagnant'

def retrieve_pos(time):
    rospy.wait_for_service(POSITION_SERVICE)
    try:
        positionRequest = rospy.ServiceProxy(POSITION_SERVICE, PositionInfo)
        resp = positionRequest(rospy.Time(0))
        print(rospy.Time(0)) 
        print(resp)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def retrieve_stag():
    rospy.wait_for_service(POSITION_SERVICE)
    try:
        stagRequest = rospy.ServiceProxy(STAGNANT_SERVICE, Stagnant)
        resp = stagRequest(True)
        print(resp)
    except rospy.ServiceException as e:
        print("Stag Service call failed: %s"%e)

if __name__ == "__main__":
    # retrieve_stag()
    # time.sleep(2)
    retrieve_pos(rospy.Time(0))