#!/usr/bin/python3

# import serial
import rospy
import math
import numpy
from geometry_msgs.msg import Twist
import RPi.GPIO as gpio
import threading

# stm = serial.Serial(port = "/dev/ttyAMA0", baudrate = 115200)

# def send_data(linxpwm, angzpwm):

#       print ("left", (linxpwm - angzpwm) / 2)
#       print ("right", (linxpwm + angzpwm) / 2)
        # stm.write(linxpwm)
        # stm.write(angzpwm)


gpio.setmode(gpio.BOARD)
left_dir_pin = 11 #29
right_dir_pin = 12 #31
left_pwm_pin =  13 #33
right_pwm_pin = 32 #35
gpio.setup(left_dir_pin, gpio.OUT)
gpio.setup(right_dir_pin, gpio.OUT)
gpio.setup(left_pwm_pin, gpio.OUT)
gpio.setup(right_pwm_pin, gpio.OUT)


#def send_pwm(vel_L_pwm, direction_L, vel_R_pwm, direction_R):
#    ls=gpio.PWM(left_pwm_pin, vel_L_pwm)
#    rs=gpio.PWM(right_pwm_pin, vel_R_pwm)
#    gpio.output(left_dir_pin, direction_L)
#    gpio.output(right_dir_pin,direction_R)
#    ls.start(50)
#    rs.start(50)

def left_pwm(vel_L_pwm, direction_L):
	ls=gpio.PWM(left_pwm_pin, vel_L_pwm)
	gpio.output(left_dir_pin, direction_L)
	ls.start(10)
	
def right_pwm(vel_R_pwm, direction_R):
	rs=gpio.PWM(right_pwm_pin, vel_R_pwm)
	gpio.output(right_dir_pin, direction_R)
	rs.start(10)

def calc_lr_vels(lin_vel_x, ang_vel_z, wheel_sep):
    d = wheel_sep
    direction_L = 1
    direction_R = 1
    vel_L = lin_vel_x - ang_vel_z * d/2
    vel_R = lin_vel_x + ang_vel_z * d/2

    vel_L_pwm = vel_L * 1023 / 0.975
    if(vel_L_pwm > 255):
        vel_L_pwm = 250

    vel_R_pwm = vel_R * 1023 / 0.975
    if(vel_R_pwm > 255):
        vel_R_pwm = 1

    if (vel_L_pwm < 0):
        direction_L = 0
        vel_L_pwm = math.fabs(vel_L_pwm)

    if (vel_R_pwm < 0):
        direction_R = 1
        vel_R_pwm = math.fabs(vel_R_pwm)

    print ("formula1 left", vel_L_pwm)
    print ("formula1 left direction", direction_L)
    print ("\n")
    print ("formula1 right", vel_R_pwm)
    print ("formula1 right direction", direction_R)
    print ("\n")
    
    left = threading.Thread(target=left_pwm(vel_L_pwm, direction_L), name='left_pwm')
    right = threading.Thread(target=right_pwm(vel_R_pwm, direction_R), name='right_pwm')
   
    left.start()
    right.start()
    
    left.join()
    right.join()
    #send_pwm(vel_L_pwm, direction_L, vel_R_pwm, direction_R)

def callback_vel(msg):
    lin_vel_x = msg.linear.x
    ang_vel_z = msg.angular.z
    d = 0.6

    calc_lr_vels(lin_vel_x, ang_vel_z, d)

def sub_vel():
    rospy.init_node('subscriber', anonymous = True)
    rate = rospy.Rate(200)

    while not rospy.is_shutdown():
        try:
            rospy.Subscriber("/cmd_vel", Twist, callback_vel)
            rate.sleep()
        # stm.write('b')
        except rospy.ROSInterruptException:
            break

if __name__ == '__main__':
   sub_vel()

