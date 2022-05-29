//pwm_direct from rpi

#!/usr/bin/python3
# import serial
import rospy
import time
import RPi.GPIO as gpio

gpio.setmode(gpio.BOARD)
left_dir_pin = 11 #29
right_dir_pin = 12 #31
left_pwm_pin =  13 #33
right_pwm_pin = 15 #35
gpio.setup(left_dir_pin, gpio.OUT)
gpio.setup(right_dir_pin, gpio.OUT)
gpio.setup(left_pwm_pin, gpio.OUT)
gpio.setup(right_pwm_pin, gpio.OUT)

pwm_l = 0
pwm_r = 0

ls = gpio.PWM(left_pwm_pin, 1000)
rs = gpio.PWM(right_pwm_pin, 1000)
while not rospy.is_shutdown():
    try:
        file1 = open("value.txt", "r")
        a = file1.readline()
        print(a)
        duty_cycle = a.split(" ")
        print(duty_cycle[0])
        if duty_cycle[0] == '' or duty_cycle[1] == '':
            continue
        else:
            gpio.output(left_dir_pin, 0)
            gpio.output(right_dir_pin, 1)
            ls.start(float(duty_cycle[0]))
            rs.start(float(duty_cycle[1]))
            time.sleep(0.1)
    except KeyboarInterrupt():
        gpio.output(left_dir_pin, 0)
        gpio.output(right_dir_pin, 0)
        gpio.output(left_pwm_pin, 0)
        gpio.output(right_pwm_pin, 0)
        break

