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
#include "rover/pcl_processing.h"
#include "rover/obstacle_info.h"
#include "rover/Centroid.h"

class clustering{
    private:
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Publisher pub_marker;
    ros::Publisher pub_OBB;
    ros::Publisher pub_obs_info;
   

    public:
    clustering(ros::NodeHandle *nh){
        sub = nh->subscribe("/Lidar_3d", 1, &clustering::callback, this);
        pub = nh->advertise<sensor_msgs::PointCloud2> ("/pcl_clustering", 1);
        pub_marker = nh->advertise<visualization_msgs::MarkerArray> ("/vis2", 1);
        pub_OBB = nh->advertise<rover::pcl_processing>("/OBB", 1);
        pub_obs_info = nh->advertise<rover::obstacle_info>("/obs_info", 1);
       
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr& input);
 
    };

void clustering::callback(const sensor_msgs::PointCloud2ConstPtr& input)
{  
    // Pointcloud Containers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);


    

    //  for markers
    uint32_t shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::MarkerArray marker_array_msg;
    std::vector<visualization_msgs::Marker> markerArray;

    // vector for storing multiple pointclouds
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>clouds;
    std::vector<Eigen::Vector4f>CentroidArray;
  
    // new method
    pcl::PointXYZRGB min_point_OBB;
    pcl::PointXYZRGB max_point_OBB;
    pcl::PointXYZRGB position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    std::vector<pcl::PointXYZRGB>min_point_OBB_array;
    std::vector<pcl::PointXYZRGB> max_point_OBB_array;
    std::vector<pcl::PointXYZRGB>position_OBB_array;
    std::vector<Eigen::Quaternionf>rotation_matrix_array;

    pcl::fromROSMsg(*input, *cloud);
    
    //  extracting indices
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);


    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud_f);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_f);

    // Using Euclidean Cluster Extraction
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.13); 
    ec.setMinClusterSize (15);
    ec.setMaxClusterSize (200000000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_f);
    ec.extract (cluster_indices);


    cloud_cluster->header.frame_id = "lidar";

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){

            // int size = cluster_indices.size();
            // int size_ind = it->indices.size();
    
            // std::cout<<std::endl<<cluster_indices[size-1].indices[2];
            cloud_cluster->push_back ((*cloud_f)[*pit]);
            sourceCloud->push_back((*cloud_f)[*pit]);

        }
    
        j++;

        // Computing Centroid
        Eigen::Vector4f Centroid;
        pcl::compute3DCentroid(*sourceCloud, Centroid);
        CentroidArray.push_back(Centroid);


     

        
        int size = sourceCloud->size();
        sourceCloud->points[0].g = 255;
        sourceCloud->points[0].b = 0;
        sourceCloud->points[0].r = 0;
        sourceCloud->points[size-1].g = 0;
        sourceCloud->points[size-1].b = 255;
        sourceCloud->points[size-1].r = 0;
        sourceCloud->header.frame_id = cloud->header.frame_id;

        if(j==1){
        pub.publish(sourceCloud);
        }
      
   
        // Computing bounding box dimensions
        pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature;
        feature.setInputCloud(sourceCloud);
        feature.compute();

        feature.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
        min_point_OBB_array.push_back(min_point_OBB);
        max_point_OBB_array.push_back(max_point_OBB);
        position_OBB_array.push_back(position_OBB);

        Eigen::Quaternionf quat (rotational_matrix_OBB);
        rotation_matrix_array.push_back(quat);

        // publishing all data
        rover::pcl_processing data;
        data.header.frame_id = "lidar";
        data.id = j;
        data.quat_OBB.x = quat.x();
        data.quat_OBB.y = quat.y();
        data.quat_OBB.z = quat.z();
        data.quat_OBB.w = quat.w();
        data.max_point_OBB.x = max_point_OBB.x;
        data.max_point_OBB.y = max_point_OBB.y;
        data.max_point_OBB.z = max_point_OBB.z;
        data.min_point_OBB.x = min_point_OBB.x;
        data.min_point_OBB.y = min_point_OBB.y;
        data.min_point_OBB.z = min_point_OBB.z;
        data.position_OBB.x = position_OBB.x;
        data.position_OBB.y = position_OBB.y;
        data.position_OBB.z = position_OBB.z;
        pub_OBB.publish(data);
        
        

        // computing obstacle projection
        float theta_min, theta_max, obstacle_projection;
        theta_min = atan(sourceCloud->points[3].x / sourceCloud->points[3].y) * (180 / M_PI);
        theta_max = atan(sourceCloud->points[size-1].x / sourceCloud->points[size-1].y) * (180 / M_PI);

        if(sourceCloud->points[3].y<0){
            theta_min += 180;
        }

        if(sourceCloud->points[size-1].y<0){
            theta_max += 180;
        }

        obstacle_projection = fabs(theta_min-theta_max);
        
        std::cout<<"\nAngle of cluster " << j << " is " << theta_max <<"  "<< theta_min  << std::endl;
        std::cout<<"\nAngle of cluster " << j << " is " << obstacle_projection << std::endl;

        rover::obstacle_info obs;

        obs.header.frame_id = "lidar";
        obs.centroidX = Centroid.x();
        obs.centroidY = Centroid.y();
        obs.centroidZ = Centroid.z();
        obs.id = j;
        obs.thetaMin = theta_min;
        obs.thetaMax = theta_max;
        obs.obstacleProjection = obstacle_projection;

        pub_obs_info.publish(obs);

} 
   
    
  
    std::cout<<std::endl<<"Number of clusters: "<<j<<std::endl;

    for(int i=0; i<j; i++){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "lidar";
        marker.header.stamp = ros::Time::now();
        marker.ns = "obstacles";
        marker.id = i;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = position_OBB_array[i].x;
        marker.pose.position.y = position_OBB_array[i].y;
        marker.pose.position.z = position_OBB_array[i].z;
        marker.pose.orientation.x = rotation_matrix_array[i].x();
        marker.pose.orientation.y = rotation_matrix_array[i].y();
        marker.pose.orientation.z = rotation_matrix_array[i].z();
        marker.pose.orientation.w = rotation_matrix_array[i].w();
        marker.scale.x = (max_point_OBB_array[i].x-min_point_OBB_array[i].x)*1.5;
        marker.scale.y = (max_point_OBB_array[i].y-min_point_OBB_array[i].y)*1.5;
        marker.scale.z = (max_point_OBB_array[i].z-min_point_OBB_array[i].z)*2;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5;

        marker.lifetime = ros::Duration();
        marker_array_msg.markers.push_back(marker);

    }
       
        pub_marker.publish(marker_array_msg);
}



int main(int argc, char** argv){
    // initialising node
    ros::init(argc, argv, "pcl_cluster_2");
    ros::NodeHandle nh;
    clustering obj =  clustering(&nh);
    ros::spin();


}