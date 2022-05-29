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
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <pcl/features/moment_of_inertia_estimation.h>
#include "rover/obstacle_info.h"
#include <dynamic_reconfigure/server.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>


class clustering{
    private:
    ros::Subscriber sub_stereocam_;
    ros::Publisher pub_clusters_;
    ros::Publisher pub_obs_info_;
    ros::Publisher pub_test_;
   

    public:
    clustering(ros::NodeHandle *nh){
        sub_stereocam_ = nh->subscribe("/camera/depth/color/points", 1, &clustering::callback, this);
        pub_clusters_= nh->advertise<sensor_msgs::PointCloud2> ("/pcl_clustering", 1);
        pub_obs_info_ = nh->advertise<rover::obstacle_info>("/obs_info", 1);
        pub_test_= nh->advertise<sensor_msgs::PointCloud2> ("/pcl_sourcecloud", 1);
       
    }
    

    void callback(const sensor_msgs::PointCloud2ConstPtr& input);

    void decideFreeSector(std::vector<float> thetaMin, std::vector<float> thetaMax, int numberOfClusters);
 
    };

void clustering::callback(const sensor_msgs::PointCloud2ConstPtr& input)
{  
    // Pointcloud Containers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr noiseless_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr passthrough_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // vector for storing multiple pointclouds
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>clouds;
    std::vector<Eigen::Vector4f>CentroidArray;
    std::vector<float>thetaMinArray;
    std::vector<float>thetaMaxArray;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  
    pcl::fromROSMsg(*input, *cloud);
    
    // passthrough
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 2.0);
    pass.filter (*passthrough_cloud);

    // noise removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> st;
    st.setInputCloud (passthrough_cloud);
    st.setMeanK (50);
    st.setStddevMulThresh (1.0);
    st.filter (*noiseless_cloud);

    // voxel grid
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (noiseless_cloud);
    sor.setLeafSize (0.06f, 0.06f, 0.06f);
    sor.filter (*voxel_cloud);

    
    
    // ground plane elimination
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.03);
    seg.setMaxIterations (1000);
    Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0);
    seg.setAxis(axis);
    seg.setEpsAngle(0.523599);
    seg.setInputCloud (voxel_cloud);
    seg.segment (*inliers, *coefficients);


    // extracting indices
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (voxel_cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud_f);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_f);

    // Using Euclidean Cluster Extraction
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.5); 
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (10000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_f);
    ec.extract (cluster_indices);


    cloud_cluster->header.frame_id = "camera_link";
    passthrough_cloud->header.frame_id = "camera_link";

 

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){

            cloud_cluster->push_back ((*cloud_f)[*pit]);
            sourceCloud->push_back((*cloud_f)[*pit]);

        }
        
        j++;

        // Computing Centroid
        Eigen::Vector4f Centroid;
        pcl::compute3DCentroid(*sourceCloud, Centroid);
        CentroidArray.push_back(Centroid);
        std::cout<<"\n------------------\n";
        std::cout<<"\nThe centroid is "<<Centroid<<std::endl;

        int size = sourceCloud->size();
        sourceCloud->points[0].g = 255;
        sourceCloud->points[0].b = 0;
        sourceCloud->points[0].r = 0;
        sourceCloud->points[size-1].g = 0;
        sourceCloud->points[size-1].b = 255;
        sourceCloud->points[size-1].r = 0;
       
        // sourceCloud->header.frame_id = "rs";

        if(j==1){
            pub_test_.publish(sourceCloud);
        }
        // computing obstacle projection
        float theta_min, theta_max, obstacle_projection;
        theta_min = atan(sourceCloud->points[0].x / sourceCloud->points[0].y) * (180 / M_PI);
        theta_max = atan(sourceCloud->points[size-1].x / sourceCloud->points[size-1].y) * (180 / M_PI);

        // if(sourceCloud->points[0].y<0){
        //     theta_min += 180;
        // }

        // if(sourceCloud->points[size-1].y<0){
        //     theta_max += 180;
        // }

        obstacle_projection = fabs(theta_min-theta_max);
        
        std::cout<<"\nMax Min Angles of cluster " << j << " is " << theta_max <<"  "<< theta_min << std::endl;
        std::cout<<"\nAngle of cluster " << j << " is " << obstacle_projection << std::endl;
        
        thetaMinArray.push_back(theta_min);
        thetaMaxArray.push_back(theta_max);

        // publishing all the data about the obstacles
        rover::obstacle_info obs;

        obs.header.frame_id = "lidar";
        obs.centroidX = Centroid.x();
        obs.centroidY = Centroid.y();
        obs.centroidZ = Centroid.z();
        obs.id = j;
        obs.thetaMin = theta_min;
        obs.thetaMax = theta_max;
        obs.obstacleProjection = obstacle_projection;

        pub_obs_info_.publish(obs);

} 
   
    pub_clusters_.publish(cloud_cluster);
  
    std::cout<<std::endl<<"Number of clusters: "<<j<<std::endl;
    std::cout<<"\n------------------\n";

    //clustering::decideFreeSector(thetaMinArray, thetaMaxArray, j);
    thetaMaxArray.clear();
    thetaMinArray.clear();

}






int main(int argc, char** argv){
    // initialising node
    ros::init(argc, argv, "pcl_cluster_2");
    ros::NodeHandle nh;
    clustering obj =  clustering(&nh);
    ros::spin();


}