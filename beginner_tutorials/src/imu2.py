#!/usr/bin/env python3 

import rospy 
import math
from sensor_msgs.msg import Imu

def imu_callback(msg):

    yaw = msg.orientation.z

    if yaw<0:
        yaw = yaw + 90
    elif yaw > 0:
        yaw = yaw - 90
    else:
        yaw = yaw
        
    msg.orientation.z = yaw
    print (msg)
    pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("imu", anonymous = True)
    pub = rospy.Publisher("/imu_publisher", queue_size = 10)
    rospy.Subscriber("/imu", Imu, imu_callback)
