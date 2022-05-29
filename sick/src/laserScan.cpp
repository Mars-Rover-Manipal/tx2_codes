#include <ros/ros.h>
#include <iostream>
#include <string>
#include "math.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class traverse
{
    private:
        ros::Subscriber sub_lidar;
        ros::Publisher pub_vel;
        ros::NodeHandle nh;
        int totalsamples;
        int required_range;
        int samples_taken;
        int sectors_;
        int maxrange;
        std::vector<float> samples_;
        geometry_msgs::Twist vel_;
        int threshold = 1;
    
    public:
        traverse();
        void clbk_laser(const sensor_msgs::LaserScan& msg);
        void obs_avoid();
        void align();
};

traverse::traverse()
{
    pub_vel = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 10); 
    sub_lidar = nh.subscribe("/laser/pcl", 1, &traverse::clbk_laser, this); 
//	sleep(7);
	ros::spin();
}

float find_min_element(std::vector<float> vec,int start_index, int end_index)
{
    int size_vec = vec.size();
    float min_element = 1000;
    for(int i=start_index;i<end_index;i++)
    {
        if(vec[i]<min_element)
        {
            min_element = vec[i];
        }
    }

    return min_element;
}

void traverse::clbk_laser(const sensor_msgs::LaserScan& msg)
{
    totalsamples = msg.ranges.size();
    required_range = 120;
    samples_taken = (totalsamples*required_range)/275;
    sectors_ = 3;
    //std::cout<<"\n";
    int Iter = samples_taken/sectors_;
    int Range = 310;
    // maxrange = 5;
    
    for(int i=0;i<sectors_;i++)
    {
        samples_.push_back(find_min_element(msg.ranges,Range,Range+Iter));
        Range += Iter;
    }
    //ROS_INFO(samples_);
	//std::cout<<samples_[0]<<std::endl;
	//std::cout<<samples_[1]<<std::endl;
	//std::cout<<samples_[2]<<std::endl;
    if(samples_[0]>threshold && samples_[1]>threshold && samples_[2]>threshold)
    {
        align();
    }
    else 
    {
        obs_avoid();
    }
    samples_.clear();
}

void traverse::obs_avoid()
{
    if(samples_[0]<samples_[2])
    {
        vel_.linear.x = 0;
        vel_.angular.z = 0.55;
//		std::cout<<"right";
		pub_vel.publish(vel_);
    }
    else if(samples_[0]>samples_[2])
    {
        vel_.linear.x = 0;
        vel_.angular.z = -0.55;
//		std::cout<<"left";
		pub_vel.publish(vel_);
    }
}

void traverse::align()
{
    vel_.linear.x = 0.3;
    vel_.angular.z = 0;
	pub_vel.publish(vel_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "LaserScan");
    traverse obj = traverse();
}

