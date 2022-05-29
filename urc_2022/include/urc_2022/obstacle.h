#include "iostream"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "math.h"
#include "tf/transform_datatypes.h"
#include <string>
#include <boost/geometry.hpp>
#include "sensor_msgs/LaserScan.h"
#include <bits/stdc++.h>

namespace obstacle_p
{
	class obstacle
	{
		public:
			ros::NodeHandle nh;
			ros::Publisher pub_vel_;
			ros::Subscriber sub_lidar_;
			//Obstacle Avoidance Parameters 
            		int totalsamples;
            		int required_range;
            		int samples_taken;
            		int sectors_;
            		std::vector<float> samples_;
            		int threshold;
            		geometry_msgs::Twist vel_;
            		//Caller function 
            		obstacle(ros::NodeHandle nh,ros::Publisher pub_vel_); 
            		void caller_obs();
            		void laserCallback(const sensor_msgs::LaserScan& msg);
            		float find_min_element(std::vector<float> vec,int start_index, int end_index);
            		void decision();
	};
}
