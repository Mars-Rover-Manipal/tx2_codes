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
		self.regions = { 
			'right': 10, 
			'front': 10, 
			'left': 10
		}
	def clbk_laser(self,msg):
		msg.ranges = list(msg.ranges)
		for i in range(len(msg.ranges)):
			if msg.ranges[i] == 0:
				msg.ranges[i] = 200 
		self.regions = { 
			'right': min(min(msg.ranges[309:463]), 50),
			'front': min(min(msg.ranges[464:618]), 50),
			'left':  min(min(msg.ranges[619:775]), 50) 
		}
	def euler(self,msg):
		self.yaw=msg.orientation.z


	def obs_avoid(self):

		if(self.regions['left'] < self.regions['right']):
			print("left")
			self.data.linear.x = 0 
			self.data.angular.z = 0.3 
		else:
			print("right")
			self.data.linear.x = 0 
			self.data.angular.z = -0.3
		rospy.loginfo(self.data)
	
	def align(self):
		self.data.linear.x = 0.5 
		self.data.angular.z = 0 

if __name__ == '__main__':
	
	ob = traverse()
	rospy.init_node('obs_traversal', anonymous=True)

	## Rate
	rate = rospy.Rate(10)

	## Publisher Cmd_vel
	pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)

	while not rospy.is_shutdown():

		## Lidar Scanner Topic
		rospy.Subscriber('/scan', LaserScan, ob.clbk_laser)

		## Logger for Regions
		#rospy.loginfo(ob.regions)

		## If no obstacle
		if ob.regions['front'] > 5.0 and ob.regions['left'] > 5.0 and ob.regions['right'] > 5.0:
			ob.align()
		else: ## Obstacle Exists
			ob.obs_avoid()

		pub.publish(ob.data)
		rate.sleep()

