#!/usr/bin/env python3
import serial
import rospy
from sensor_msgs.msg import Imu
if __name__ == '__main__':
    rospy.init_node('imu_bno', anonymous=True)
    imu_msg = Imu()
    pub = rospy.Publisher('imu', Imu, queue_size=10)
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    ser.flush()
    while not rospy.is_shutdown():
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').rstrip()
                if(line.__str__() == ''):
                    continue
                print(line)
                imu_msg.orientation.z = float(line.__str__()) 
                pub.publish(imu_msg)
        except rospy.ROSInterruptException:
            break
