#!usr/bin/python3

import rospy
import time
import numpy
from geometry_msgs.msg import Twist

def publisher():
    vel = Twist()
    rate = rospy.Rate(100)
    c = 0.05
    while not rospy.is_shutdown():
        #vel.angular.z = 0
        vel.linear.x = 1  #To be Changed
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = -0.55 + c #To be Changed
        print(vel)
        c=c+0.0001625
        pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('chala')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

    

