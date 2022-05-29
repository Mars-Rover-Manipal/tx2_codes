#!/usr/bin python3

import time
import socket
import math
#import Jetson.GPIO as gpio
import rospy
from geometry_msgs.msg import Twist

# gpio.setmode(gpio.BOARD)
# left_dir_pin = 37 #29
# right_dir_pin = 31 #31
# left_pwm_pin =  33 #33
# right_pwm_pin = 29 #35

# gpio.setup(left_dir_pin, gpio.OUT)
# gpio.setup(right_dir_pin, gpio.OUT)
# gpio.setup(left_pwm_pin, gpio.OUT)
# gpio.setup(right_pwm_pin, gpio.OUT)

# ls = gpio.PWM(left_pwm_pin, 10000)
# rs = gpio.PWM(right_pwm_pin, 10000)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
HOST = "192.168.1.7"
PORT = 5005 
s.connect((HOST, PORT))

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
    print(linear)

    vel_L = linear - angular * 0.35
    vel_R = linear + angular * 0.35

    dc_L = vel_L * 100 / 0.975
    dc_R = vel_R * 100 / 0.975

    if math.fabs(dc_L) > 100:
        dc_L = 100
    if math.fabs(dc_R) > 100:
        dc_R = 100

    print(dc_L)
    print(dc_R)

    if vel_L < 0:
        dir_L = 0
        dc_L = math.fabs(vel_L)
 
    if vel_R < 0:
        dir_R = 0
        dc_R = math.fabs(vel_L)
 
        # gpio.output(left_dir_pin, dir_L)
        # gpio.output(right_dir_pin, dir_R)
        # ls.start(dc_L)
        # rs.start(dc_R)
    
    data = "l"+dc_L+dir_L+"r"+dc_R+dir_R
    print(data)
    s.send(data.encode('utf-8'))

    if name == 'main':
        rospy.init_node('subscriber', anonymous = True)
        while not rospy.is_shutdown():
            try:
                rospy.Subscriber("/cmd_vel", Twist, callback_vel)
                time.sleep(0.5)
            except rospy.ROSInterruptException:
                # gpio.PWM(right_pwm_pin, 0)
                # gpio.PWM(left_pwm_pin,0)
                # gpio.output(left_dir_pin,0)
                # gpio.output(right_dir_pin,0)
                break
