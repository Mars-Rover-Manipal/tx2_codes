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
        float* a,b,c;
        geometry_msgs::Twist vel_;
        int threshold = 5;
    public:
        traverse();
        void clbk_laser(const sensor_msgs::LaserScan& msg);
        void obs_avoid();
        void align();
};

traverse::traverse()
{
    sub_lidar = nh.subscribe("/laser/pcl", 1, &traverse::clbk_laser, this);
    pub_vel = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
    ros::spin();
}


void traverse::clbk_laser(const sensor_msgs::LaserScan& msg)
{
    a = min_element(msg.ranges.begin()+310,msg.ranges.begin()+470);
    *a = std::min(*a,10.0);
    b = min_element((msg.ranges.begin()+471),(msg.ranges.begin()+631));
    *b = std::min(*b,10.0);
    c = *min_element((msg.ranges.begin()+632),(msg.ranges.begin()+792));
    *c = std::min(*c,10.0);


    if(a>threshold && b>threshold && c>threshold)
    {
        align();
    }
    else
    {
        obs_avoid();
    }
}

void traverse::obs_avoid()
{
    if(a<c)
    {
        vel_.linear.x = 0;
        vel_.angular.z = -0.55;
    std::cout<<"1";
    pub_vel.publish(vel_);
    }
    else if(a>c)
    {
        vel_.linear.x = 0;
        vel_.angular.z = 0.55;
    std::cout<<"2";
    pub_vel.publish(vel_);
    }
}

void traverse::align()
{
    vel_.linear.x = 0.5;
    vel_.angular.z = 0;
  pub_vel.publish(vel_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "LaserScan");
    traverse obj = traverse();
}


