#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import pyproj
import math

class traverse:
	def __init__(self):
		self.yaw = 0
		self.data = Twist()
		self.regions = {}
	
	def clbk_laser(self, msg):
        
		self.regions = {
			'left': min(min(msg.ranges[240:479]), 50),
			'front': min(min(msg.ranges[480:719]), 50),
			'right':  min(min(msg.ranges[720:959]), 50),
		}
	#def euler(self,msg):
	#	self.yaw=msg.orientation.z

	def obs_avoid(self):
		if self.regions['front'] > 5 and self.regions['left'] > 5 and self.regions['right'] > 5:
		#no obstacle
			self.data.linear.x = 0.5
			self.data.angular.z = 0
		elif self.regions['front'] < 5 and self.regions['left'] > 5 and self.regions['right'] > 5:
		#obstacle in the front
			self.data.linear.x = 0
			self.data.angular.z = 0.5
		elif self.regions['front'] > 5 and self.regions['left'] > 5 and self.regions['right'] < 5:
		#obstacle on right
			self.data.linear.x = 0
			self.data.angular.z = 0.5
		elif self.regions['front'] > 5 and self.regions['left'] < 5 and self.regions['right'] > 5:
		#obstacle on left
			self.data.linear.x = 0
			self.data.angular.z = -0.5
		elif self.regions['front'] < 5 and self.regions['left'] > 5 and self.regions['right'] < 5:
		#obstacle on front and right
			self.data.linear.x = 0
			self.data.angular.z = 0.5
		elif self.regions['front'] < 5 and self.regions['left'] < 5 and self.regions['right'] > 5:
		#obstacle on front and left
			self.data.linear.x = 0
			self.data.angular.z = -0.5
		elif self.regions['front'] < 5 and self.regions['left'] < 5 and self.regions['right'] < 5:
		#obstacle on all three regions
			self.data.linear.x = 0
			self.data.angular.z = -0.5
		elif self.regions['front'] > 5 and self.regions['left'] < 5 and self.regions['right'] < 5:
		#obstacle on left and right
			self.data.linear.x = 0.5
			self.data.angular.z = 0
		else:
			state_description = 'unknown case'
	
	def align(self):
		self.data.linear.x = 0.5
		self.data.angular.z = 0

	def talker(self):
		rospy.init_node('traversal', anonymous=True)
		rate=rospy.Rate(10)
		pub = rospy.Publisher('cmd_vel',Twist, queue_size=10)
		while True:
			rospy.Subscriber('/scan', LaserScan, self.clbk_laser)
#			rospy.Subscriber('/imu',Imu,self.euler)
			rate.sleep()
			rospy.loginfo(self.regions)
			if self.regions['front'] > 5.0 and self.regions['left'] > 5.0 and self.regions['right'] > 5.0:
				self.align()
			else:
				self.obs_avoid()
			pub.publish(self.data)
		
if __name__ == '__main__':
	ob=traverse()
	try:
		ob.talker()
	except rospy.ROSInterruptException:
		exit

