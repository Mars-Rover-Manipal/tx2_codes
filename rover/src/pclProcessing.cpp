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
#include "rover/free_sectors.h"
#include "rover/num_clusters.h"



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
    ros::Publisher pub_obs_info_;
    ros::Publisher pub_free_sectors_;
    ros::Subscriber sub_azimuth_;
    ros::Publisher pub_num_clusters_;
   

    public:
    clustering(ros::NodeHandle *nh){
        sub_lidar_ = nh->subscribe("/Lidar_3d", 1, &clustering::callback, this);
        pub_clusters_= nh->advertise<sensor_msgs::PointCloud2> ("/pcl_clustering", 1);
        pub_obs_info_ = nh->advertise<rover::obstacle_info>("/obs_info", 1);
        pub_free_sectors_ = nh->advertise<rover::free_sectors>("/free_sectors", 1);
        pub_num_clusters_ = nh->advertise<rover::num_clusters>("/num_clusters", 1);
    
    }
    
    
    void callback(const sensor_msgs::PointCloud2ConstPtr& input);

    void decideFreeSector(std::vector<thetaPlusCentroid> thetaMin, std::vector<thetaPlusCentroid> thetaMax, int numberOfClusters);
 
    };

void clustering::callback(const sensor_msgs::PointCloud2ConstPtr& input)
{  
    // Pointcloud Containers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

    // vector for storing multiple pointclouds

    std::vector<Eigen::Vector4f>CentroidArray;
    std::vector<float>DistanceArray;
    std::vector<thetaPlusCentroid>thetaMinArray_222to360;
    std::vector<thetaPlusCentroid>thetaMaxArray_222to360;
    std::vector<thetaPlusCentroid>thetaMinArray_0to137;
    std::vector<thetaPlusCentroid>thetaMaxArray_0to137;
  
  
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


    cloud_cluster->header.frame_id = "cloud";

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
        float distance;
    
        pcl::compute3DCentroid(*sourceCloud, Centroid);

        distance = sqrt(pow(Centroid.x(),2) + pow(Centroid.y(),2));
        CentroidArray.push_back(Centroid);
        DistanceArray.push_back(distance);

        // printing centroid
        // std::cout<<"\n Centroid of "<<j<<" is "<<Centroid<<std::endl;
        
        int size = sourceCloud->size();
        
       // computing obstacle projection
        float theta_min, theta_max, obstacle_projection;
        theta_min = atan(sourceCloud->points[size-1].x / sourceCloud->points[size-1].y) * (180 / M_PI);
        theta_max = atan(sourceCloud->points[0].x / sourceCloud->points[0].y) * (180 / M_PI);
        
        // angle correction for Global Planning
        if(sourceCloud->points[0].y<0){
            theta_min += 90;
        }
        else if(sourceCloud->points[0].y>0){
            theta_min += 270;
        }
        

        if(sourceCloud->points[size-1].y<0){
            theta_max += 90;
        }
        else if(sourceCloud->points[0].y>0){
            theta_max += 270;
        }
        
        std::cout<<"\ntheta_min thata_max "<<theta_min<<" "<<theta_max<<std::endl;
        // computing obstacle projection
        obstacle_projection = fabs(theta_min-theta_max);
        
        // storing incoming min max according to their angular ranges
        thetaPlusCentroid temp;
        if(theta_min>=222.5 and theta_min<=360){
           temp.Centroid = Centroid;
           temp.theta = theta_min;
           thetaMinArray_222to360.push_back(temp);
    
        
        }
        else if(theta_min>=0 and theta_min<=137.5){
           temp.Centroid = Centroid;
           temp.theta = theta_min;
           thetaMinArray_0to137.push_back(temp);

        }

        if(theta_max>=222.5 and theta_max<=360){
           temp.Centroid = Centroid;
           temp.theta = theta_max;
           thetaMaxArray_222to360.push_back(temp);
    
        }
        else if(theta_max>=0 and theta_max<=137.5){
           temp.Centroid = Centroid;
           temp.theta = theta_max;
           thetaMaxArray_0to137.push_back(temp);
  
        }
       
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
   if(j!=0){
  
    std::cout<<"\n------------------\n";

      // finding minimum distance
    float minimumDistance = DistanceArray[0];
    for(int i=0; i<j; i++){
        if(DistanceArray[i] < minimumDistance){
            minimumDistance = DistanceArray[i];
        }
    } 
    
    rover::num_clusters data_num_clusters_;
    data_num_clusters_.numOfClusters = j;
    data_num_clusters_.minDistance = minimumDistance;
    pub_num_clusters_.publish(data_num_clusters_);

    std::cout<<std::endl<<"Number of clusters: "<<j<<std::endl;
    
    // // sorting the vectors
    // std::cout<<"BEFORE SORTING";

    // for(int i=0; i<2; i++){
    //     std::cout<<thetaMinArray_0to137[i].theta<<" "<<thetaMinArray_0to137[i].Centroid;
    // }

    sort(thetaMinArray_222to360.begin(), thetaMinArray_222to360.end(), compareByValue);
    sort(thetaMinArray_0to137.begin(), thetaMinArray_0to137.end(), compareByValue);
    sort(thetaMaxArray_222to360.begin(), thetaMaxArray_222to360.end(), compareByValue);
    sort(thetaMaxArray_0to137.begin(), thetaMaxArray_0to137.end(), compareByValue);
    
    // std::cout<<"AFTER SORTING";
    
    // for(int i=0; i<2; i++){
    //     std::cout<<thetaMinArray_0to137[i].theta<<" "<<thetaMinArray_0to137[i].Centroid;
    // }

    // concatenating the 2 vectors

    std::vector<thetaPlusCentroid> thetaMinArray;
    std::vector<thetaPlusCentroid> thetaMaxArray;
    
    thetaMinArray = concat(thetaMinArray_222to360, thetaMinArray_0to137);
    thetaMaxArray = concat(thetaMaxArray_222to360, thetaMaxArray_0to137);
    
    std::cout<<"\n``````````````````\n"; 
    for(int i=0; i<j; i++){
        std::cout<<std::endl<<"\nMin Angle "<<thetaMinArray[i].theta;
        std::cout<<std::endl<<" Max Angle "<<thetaMaxArray[i].theta;
    }
    std::cout<<"\n``````````````````\n"; 
    
    if(j!=0){
    clustering::decideFreeSector(thetaMinArray, thetaMaxArray, j);
    }

    thetaMaxArray.clear();
    thetaMinArray.clear();
   }
}


void clustering::decideFreeSector(std::vector<thetaPlusCentroid> thetaMin, std::vector<thetaPlusCentroid> thetaMax, int numberOfClusters){


    rover::free_sectors data;

    std::vector<float>thetaMinFree;
    std::vector<float>thetaMaxFree;
    std::vector<std::array<Eigen::Vector4f, 2> > Centroids;
    std::array<Eigen::Vector4f, 2> temp;
   
    
    if(thetaMin[0].theta<222.5){
            thetaMin[0].theta = 222.5;
        }

    thetaMinFree.push_back(222.5);
    thetaMaxFree.push_back(thetaMin[0].theta);
    
    temp[0] = Eigen::Vector4f::Zero();
    temp[1] = thetaMin[0].Centroid;
    Centroids.push_back(temp);
    
    
       
    for(int i=0; i<numberOfClusters-1; i++){
        thetaMinFree.push_back(thetaMax[i].theta);
        thetaMaxFree.push_back(thetaMin[i+1].theta);

        temp[0] = thetaMax[i].Centroid;
        temp[1] = thetaMin[i+1].Centroid;
        Centroids.push_back(temp);
    }

    thetaMinFree.push_back(thetaMax[numberOfClusters-1].theta);
    thetaMaxFree.push_back(137.5);

    temp[0] = thetaMax[numberOfClusters-1].Centroid;
    temp[1] = Eigen::Vector4f::Zero();
    Centroids.push_back(temp);


        std::cout<<"\n~~~~~~~~~~~~~~~~~~\n"; 
        for(int i=0; i<numberOfClusters+1; i++){
        std::cout<<"\nTheta Min " <<thetaMinFree[i]<<" ";
        std::cout<<"Theta Max " <<thetaMaxFree[i];
        std::cout<<" Centroid_left "<<Centroids[i][0];
        std::cout<<" Centroid_right "<<Centroids[i][1]<<"\n";
        } 
        std::cout<<"\n~~~~~~~~~~~~~~~~~~\n";  
        std::cout<<"\n------------------\n";
    
    // finding cost aaray
    std::vector<float> costArray;
    for(int i=0; i<numberOfClusters+1; i++){
        float mid_angle_ = (thetaMaxFree[i] + thetaMinFree[i])/2;
        float cost = fabs(mid_angle_ - 300);
        costArray.push_back(cost);
    }
    
    for(int i=0; i<numberOfClusters+1; i++){
    std::cout<<"Cost Array"<<costArray[i]<<std::endl;
    }
    // finding the appropriate free sector
    int index = 0, minimum = costArray[0];
    
    for(int i=1; i<numberOfClusters+1; i++){
        if(costArray[i] < minimum){
            minimum = costArray[i];
            index = i;
        }
    } 
    
    std::cout<<"\nMinimum "<<minimum<<std::endl;

    data.thetaMinFreeSector = thetaMinFree[index];
    data.thetaMaxFreeSector = thetaMaxFree[index];
    data.thetaMin_Centroid.x = Centroids[index][0].x();
    data.thetaMin_Centroid.y = Centroids[index][0].y();
    data.thetaMin_Centroid.z = Centroids[index][0].z();
    data.thetaMax_Centroid.x = Centroids[index][1].x();
    data.thetaMax_Centroid.y = Centroids[index][1].y();
    data.thetaMax_Centroid.z = Centroids[index][1].z();

    pub_free_sectors_.publish(data);

    std::cout<<"\nThe Appropriate free sector is "<<thetaMinFree[index]<<" "<<thetaMaxFree[index]<<" and associated centroids are "<< Centroids[index][0]<< " "<<Centroids[index][1]<<std::endl; 
   
   }

int main(int argc, char** argv){
    // initialising node
    ros::init(argc, argv, "pcl_cluster_2");
    ros::NodeHandle nh;
    clustering obj =  clustering(&nh);
    ros::spin();


}