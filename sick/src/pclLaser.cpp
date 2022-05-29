#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <math.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class pcl_obs
{
  public:

    ros::NodeHandle nh; 
    ros::Publisher pub;
    ros::Publisher pcl;
    ros::Subscriber sub;
    sensor_msgs::LaserScan laser;
    sensor_msgs::PointCloud2 points;

    void callback(const PointCloud::ConstPtr& msg);
    void caller();
};
void pcl_obs::callback(const PointCloud::ConstPtr& msg)
{
  int i = 0;

  PointCloud::Ptr msg1 (new PointCloud);
  
  msg1->header = msg->header;
  
  laser.header.frame_id = "cloud";
  
  //msg1->height = msg->width = 1;
  std::vector<float> l;
  
//  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  
  // BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
  
  for(auto pt:msg->points)
  {
    // if(i%4 == 1)
    if(i<=1100)
    {
  //      printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
        // printf("1\n");
        //msg1->points.push_back (pcl::PointXYZ(pt.x, pt.y, pt.z));
        
        float dist = sqrt(pow(pt.x,2)+pow(pt.y,2));
        if (dist == 0)
        	dist = HUGE_VALF;
        l.push_back(dist);
        pub.publish(laser);
    }
        i++;

  }
  //pcl_conversions::toPCL(ros::Time::now(), msg1->header.stamp);
  //pcl.publish (msg1);

  laser.ranges = l;
  pub.publish(laser);
}

void pcl_obs::caller()
{
  sub = nh.subscribe<PointCloud>("/cloud",1,&pcl_obs::callback,this);
  pub = nh.advertise<sensor_msgs::LaserScan> ("/laser/pcl", 1); 
  //pcl = nh.advertise<sensor_msgs::PointCloud2> ("/cloud/augmented", 1); 
//  sleep(2);	
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pclLaser");
  pcl_obs obj;
  obj.caller();
  ros::spin();
}
