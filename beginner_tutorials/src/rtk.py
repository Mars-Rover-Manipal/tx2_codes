#!/usr/bin/env python3
import serial
from sensor_msgs.msg import NavSatFix
from ublox_gps import UbloxGps
import rospy

rospy.init_node('GPS', anonymous=True)
pub = rospy.Publisher('/fix', NavSatFix, queue_size=10)
port = serial.Serial('/dev/ttyACM1', baudrate=38400, timeout=0.1)
gps = UbloxGps(port)


def run():

    try:
        print("Listening for UBX Messages")
        rtk = NavSatFix()
        while not rospy.is_shutdown():
            try:
                geo = gps.geo_coords()
                rtk.longitude = geo.lon
                rtk.latitude = geo.lat
                pub.publish(rtk)
            except rospy.ROSInterruptException():
                break

    except:
        rt.close()
        


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass


