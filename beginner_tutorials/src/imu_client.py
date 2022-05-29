#!/usr/bin/env python3

import rospy
import sys
import rosbag
import numpy
import math
import socket,traceback
from beginner_tutorials.srv import *
from std_msgs.msg import Int32, String, Float32MultiArray

def IMU_client():
    host = '192.168.1.26'
    port = 5555

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind((host, port))
    while True:
        try:
            message, address = s.recvfrom(1024)
            message=message.replace(" ","")
            message=message.split(",")
            message=list(map(float,message))
            message=numpy.array(message)
            ori_x=message[10]
            ori_y=message[11]
            ori_z=message[12]
            rospy.wait_for_service('imu_to_quat')
            try:
                imus=rospy.ServiceProxy('imu_to_quat',imu)
                resp=imus(ori_x,ori_y,ori_z)
                print("qx=",resp.qw," qy=",resp.qx," qz=",resp.qy," qw=",resp.qz)
            except rospy.ServiceException as e:
                print("Server call failed ")
        except (KeyboardInterrupt, SystemExit):
            raise
        
        except:
            traceback.print_exc()

if __name__ == "__main__":
    print("Sending data to the server")
    IMU_client()

