#!/usr/bin/python3
# import serial
import RPi.GPIO as gpio
import rospy
from geometry_msgs.msg import Twist

gpio.setmode(gpio.BOARD)
left_dir_pin = 11 #29
right_dir_pin = 12 #31
left_pwm_pin =  13 #33
right_pwm_pin = 15 #35

linear = 0
angular = 0
gpio.setup(left_dir_pin, gpio.OUT)
gpio.setup(right_dir_pin, gpio.OUT)
gpio.setup(left_pwm_pin, gpio.OUT)
gpio.setup(right_pwm_pin, gpio.OUT)
ls=gpio.PWM(left_pwm_pin, 25)
rs=gpio.PWM(right_pwm_pin, 25)

def callback_vel(msg):
	linear = msg.linear.x
	angular = msg.angular.z
	print(linear)

if __name__ == '__main__':
	rospy.init_node('subscriber', anonymous = True)
	while not rospy.is_shutdown():
		try:
			rospy.Subscriber("/cmd_vel", Twist, callback_vel)
			gpio.output(left_dir_pin, 0)
			gpio.output(right_dir_pin, 1)
			ls.start(50)
			rs.start(50)
		except rospy.ROSInterruptException:
			break


