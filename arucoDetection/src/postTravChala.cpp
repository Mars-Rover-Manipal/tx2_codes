#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"
#include <opencv2/calib3d/calib3d.hpp> 
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <armadillo>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string> 
#include "geometry_msgs/Twist.h"
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

class PostTraversal{

 private:
    ros::Subscriber sub,sub_gps;
    ros::Publisher pub_vel;
    ros::Time curr_time;
    double distance;
    double y;
    double Lat,Long;
    int id_detected;
    

  public:
   PostTraversal(){
   ROS_INFO_STREAM("Into Constructor");
   ros::NodeHandle nh;
   sub_gps = nh.subscribe("/fix", 1000, &PostTraversal::gpsCallback, this); 
   pub_vel = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
   }

	void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
		geometry_msgs::Twist Vel;
		//ROS_INFO_STREAM("Into Subscriber");
		ros::NodeHandle nh;
		Lat = msg->latitude;
		Long = msg->longitude;
		nh.getParam("/marker/y",y);
		nh.getParam("/marker_ids_detected",id_detected);
		nh.getParam("/marker/distance",distance);
		Vel.linear.x =  distance-0.6 ;
		if(distance>0.4 && id_detected!=256){
			double diff = abs(y)-0.27;
			ROS_INFO_STREAM("Difference : "<<diff);
			if(diff>0){
				Vel.angular.z = -1.5*diff;
				ROS_INFO_STREAM(Vel);
			}
			else{
				Vel.angular.z = 1.5*diff;
				ROS_INFO_STREAM("Published Velocity");
				ROS_INFO_STREAM(Vel);
			}		
		}
		else{
			Vel.linear.x = 0;
			Vel.angular.z = 0;
		}
		pub_vel.publish(Vel);
	
	}
};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "post_traversal");
  ROS_INFO_STREAM("Started Post Traversal Node");
  PostTraversal obj =  PostTraversal();
  ros::spin();
}
