
#!/usr/bin/python3

import time
import socket
import math
#import RPi.GPIO as gpio
import rospy
from geometry_msgs.msg import Twist

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

s.connect(("192.168.1.134", 5005))
linear = 0
angular = 0

vel_L = 0
vel_R = 0
pwm_L = 0
pwm_R = 0

dir_L = 1
dir_R = 1

def callback_vel(msg):
        global left_dir_pin, right_pwm_pin, left_pwm_pin, right_pwm_pin, dir_L, dir_R
        linear = msg.linear.x
        angular = msg.angular.z
       # print(linear)

        vel_L = linear - angular * 0.35
        vel_R = linear + angular * 0.35

        pwm_L = int(vel_L * 255 / 0.975)
        pwm_R = int(vel_R * 255 / 0.975)

        if math.fabs(pwm_L) > 255:
            pwm_L = 255
        if math.fabs(pwm_R) > 255:
            pwm_R = 255

       # print(pwm_L)
       # print(pwm_R)

        if vel_L < 0:
            dir_L = 0
            pwm_L = int(math.fabs(pwm_L))
        if vel_R < 0:
            dir_R = 0
            pwm_R = int(math.fabs(pwm_L))

       # print(dir_L)
       # print(dir_R)

        msg = "m" + str(pwm_R).zfill(3) + str(dir_R) + str(pwm_L).zfill(3) + str(dir_L) + "e"
#        s.sendall(bytes(msg, "utf-8"))
        data = s.recv(1024)
        print(data.decode())
        #gpio.output(left_dir_pin, dir_L)
        #gpio.output(right_dir_pin, dir_R)
        #ls.start(dc_L)
        #rs.start(dc_R)
 #       print("Left dir",dir_L)
#      print("right dir",dir_R)

if __name__ == '__main__':
        rospy.init_node('subscriber', anonymous = True)
        while not rospy.is_shutdown():
                try:
                        rospy.Subscriber("/cmd_vel", Twist, callback_vel)
                        time.sleep(0.01)
                except rospy.ROSInterruptException:
 #                       s.sendall(bytes("m00000000e", "utf-8"))
                        break

