#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/transforms.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "rover/obstacle_info.h"


namespace pclProcessing{
class clustering{
    private:
    ros::Subscriber sub_lidar_;
   
    ros::Publisher pub_obs_info_;
   

    public:
    clustering(ros::NodeHandle *nh){
        sub = nh->subscribe("/Lidar_3d", 1, &clustering::callback, this);
        pub_obs_info_ = nh->advertise<rover::obstacle_info>("/obs_info", 1);
       
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr& input);
 
    };
}

