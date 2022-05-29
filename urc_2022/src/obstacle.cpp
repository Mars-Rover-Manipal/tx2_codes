#include "urc_2022/obstacle.h"

using namespace obstacle_p;

obstacle::obstacle(ros::NodeHandle nh,ros::Publisher pub_vel_)
{
	this->nh = nh;
	this->pub_vel_ = pub_vel_;
	threshold = 1;
}

void obstacle::caller_obs()
{
	sub_lidar_ = nh.subscribe("/laser/pcl", 1000, &obstacle::laserCallback, this);         
	//ros::spin();
}


void obstacle::laserCallback(const sensor_msgs::LaserScan& msg)
{
    totalsamples = msg.ranges.size();
    required_range = 120;
    samples_taken = (totalsamples*required_range)/275;
    sectors_ = 3;
    int Iter = samples_taken/sectors_;
    int Range = 310;
    
    for(int i=0;i<sectors_;i++)
    {
        samples_.push_back(find_min_element(msg.ranges,Range,Range+Iter));
        Range += Iter;
    }
    return;
}

float obstacle::find_min_element(std::vector<float> vec,int start_index, int end_index)
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

void obstacle::decision()
{
    if(samples_[0]>threshold && samples_[1]<threshold && samples_[2]>threshold)
    {
        std::cout<<"Obstacle: Front\n";
        vel_.linear.x = 0.0;
        vel_.angular.z = 0.3; 
        pub_vel_.publish(vel_);
    }
    else if(samples_[0]<threshold && samples_[1]>threshold && samples_[2]>threshold)
    {
        std::cout<<"Obstacle: Right\n";
        vel_.linear.x = 0.0;
        vel_.angular.z = 0.3;
        pub_vel_.publish(vel_);
    }
    else if(samples_[0]>threshold && samples_[1]>threshold && samples_[2]<threshold)
    {
        std::cout<<"Obstacle: Left\n";
        vel_.linear.x = 0.0;
        vel_.angular.z = -0.3;
        pub_vel_.publish(vel_);
    }
    else if(samples_[0]<threshold && samples_[1]<threshold && samples_[2]>threshold)
    {
        std::cout<<"Obstacle: Front & Right\n";
        vel_.linear.x = 0.0;
        vel_.angular.z = 0.3;
        pub_vel_.publish(vel_);
    }
    else if(samples_[0]>threshold && samples_[1]<threshold && samples_[2]<threshold)
    {
        std::cout<<"Obstacle: Front & Left\n";
        vel_.linear.x = 0.0;
        vel_.angular.z = -0.3;
        pub_vel_.publish(vel_);
    }
    else if(samples_[0]<threshold && samples_[1]<threshold && samples_[2]<threshold)
    {
        std::cout<<"Obstacle: Front & Right & Left\n";
        vel_.linear.x = 0.0;
        vel_.angular.z = 0.3;
        pub_vel_.publish(vel_);
    }
    else if(samples_[0]<threshold && samples_[1]>threshold && samples_[2]<threshold)
    {
        std::cout<<"Obstacle: Right & Left\n";
        vel_.linear.x = 0.0;
        vel_.angular.z = 0.3;
        pub_vel_.publish(vel_);
    }
    else 
    {
        std::cout<<"Unknown Case\n";
    }
}
