#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class pcl_obs
{
  public:

    ros::NodeHandle nh; 
    ros::Publisher pub;
    ros::Subscriber sub;

    void callback(const PointCloud::ConstPtr& msg);
};
void pcl_obs::callback(const PointCloud::ConstPtr& msg)
{
  pub = nh.advertise<PLaserScan> ("laser", 1); 
  int i = 0;
  LaserScan::Ptr msg1 (new PointCloud);
  msg1->header.frame_id = "cloud";
  //msg1->height = msg->width = 1;

  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
  {
    if(i%4 == 0)
    {   
        printf("hello");
        printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
        msg1->points.push_back (pcl::PointXYZ(pt.x, pt.y, pt.z));
        pub.publish(msg1);
    }   
    i++;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  pcl_obs obj;
  obj.sub = obj.nh.subscribe<PointCloud>("/cloud", 1, &pcl_obs::callback);
  ros::spin();
}

