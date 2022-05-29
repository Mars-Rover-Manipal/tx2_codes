#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/transforms.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Bool.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <pcl/features/moment_of_inertia_estimation.h>
#include "rover/obstacle_info.h"
#include "rover/free_sectors.h"
#include "rover/num_clusters.h"


namespace pclProcessing{
template <typename T> std::vector<T> concat(std::vector<T> &a, std::vector<T> &b) {
    std::vector<T> ret = std::vector<T>();
    copy(a.begin(), a.end(), back_inserter(ret));
    copy(b.begin(), b.end(), back_inserter(ret));
    return ret;
}

struct thetaPlusCentroid{
    float theta;
    Eigen::Vector4f Centroid;
};

bool compareByValue(const thetaPlusCentroid &a, const thetaPlusCentroid &b){
    return a.theta < b.theta;
}


class clustering{
    private:
        ros::Subscriber sub_lidar_;
        ros::Publisher pub_clusters_;
        ros::Publisher pub_free_sectors_;
        ros::Subscriber sub_azimuth_;
        ros::Publisher pub_initiate_recovery_;
    
   
    public:
        clustering(ros::NodeHandle *nh);

        void callback(const sensor_msgs::PointCloud2ConstPtr& input);
        void decideFreeSector(std::vector<thetaPlusCentroid> thetaMin, std::vector<thetaPlusCentroid> thetaMax, int numberOfClusters);
 
    };




}

