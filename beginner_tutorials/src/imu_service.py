#!/usr/bin/env python3
    
from __future__ import print_function
    
from beginner_tutorials.srv import imu,imuResponse
from tf.transformations import quaternion_from_euler
import rospy
import math
import numpy as np

    
def quaternions(req):
    q = quaternion_from_euler(req.ori_x,req.ori_y,req.ori_z)
    return imuResponse(q[0],q[1],q[2],q[3])
   
def imu_data():
    rospy.init_node('imu_to_quat_server')
    s = rospy.Service('imu_to_quat', imu, quaternions)
    rospy.spin()
   
if __name__ == "__main__":
     imu_data()

