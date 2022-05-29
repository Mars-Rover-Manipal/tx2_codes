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
#include <pcl/filters/passthrough.h>


template <typename T> std::vector<T> concat(std::vector<T> &a, std::vector<T> &b) {
    std::vector<T> ret = std::vector<T>();
    copy(a.begin(), a.end(), back_inserter(ret));
    copy(b.begin(), b.end(), back_inserter(ret));
    return ret;
}
class clustering{
    private:
    ros::Subscriber sub_lidar_;
    ros::Publisher pub_clusters_;
    ros::Publisher pub_obs_info_;
    ros::Publisher pub_free_sectors_;
    ros::Subscriber sub_azimuth_;
    ros::Publisher pub_num_clusters_;
    ros::Publisher pub_test_;
   

    public:
    clustering(ros::NodeHandle *nh){
        sub_lidar_ = nh->subscribe("/cloud", 1, &clustering::callback, this);
        pub_clusters_= nh->advertise<sensor_msgs::PointCloud2> ("/pcl_clustering", 1);
        pub_obs_info_ = nh->advertise<rover::obstacle_info>("/obs_info", 1);
        pub_free_sectors_ = nh->advertise<rover::free_sectors>("/free_sectors", 1);
        pub_num_clusters_ = nh->advertise<rover::num_clusters>("/num_clusters", 1);
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr passthrough_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    // vector for storing multiple pointclouds
    std::vector<Eigen::Vector4f>CentroidArray;
    std::vector<float>DistanceArray;
    std::vector<float>thetaMinArray_222to360;
    std::vector<float>thetaMinArray_0to137;
    std::vector<float>thetaMaxArray_222to360;
    std::vector<float>thetaMaxArray_0to137;
  
    // passthrough along y
    pcl::fromROSMsg(*input, *cloud);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-2.0, 2.0);
    pass.filter (*passthrough_cloud);
    
    // passthrough along x
  
    // pcl::PassThrough<pcl::PointXYZRGB> pass1;
    // pass1.setInputCloud (passthrough_cloud);
    // pass1.setFilterFieldName ("x");
    // pass1.setFilterLimits (-0.1, 0.0);
    // pass1.filter (*passthrough_cloud);
    // passthrough_cloud->header.frame_id = "cloud";
    
    //  extracting indices
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);


    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (passthrough_cloud);
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

            if(j==0){
            cloud_f->points[*pit].r = 0;
            cloud_f->points[*pit].g = 255;
            cloud_f->points[*pit].b = 0;
            }
            if(j==1){
            cloud_f->points[*pit].r = 0;
            cloud_f->points[*pit].g = 0;
            cloud_f->points[*pit].b = 255;
            }
            if(j==2){
            cloud_f->points[*pit].r = 255;
            cloud_f->points[*pit].g = 0;
            cloud_f->points[*pit].b = 0;
            }
            if(j==3){
            cloud_f->points[*pit].r = 255;
            cloud_f->points[*pit].g = 0;
            cloud_f->points[*pit].b = 255;
            }
             if(j==4){
            cloud_f->points[*pit].r = 100;
            cloud_f->points[*pit].g = 0;
            cloud_f->points[*pit].b = 255;
            }
            cloud_cluster->push_back ((*cloud_f)[*pit]);
            sourceCloud->push_back((*cloud_f)[*pit]);

        }
        
        j++;

        // Computing Centroid
        Eigen::Vector4f Centroid;
        float distance;
    
        pcl::compute3DCentroid(*sourceCloud, Centroid);
        std::cout<<"\n The Centroid is "<<Centroid<<std::endl;
        distance = sqrt(pow(Centroid.x(),2) + pow(Centroid.y(),2));
        CentroidArray.push_back(Centroid);
        DistanceArray.push_back(distance);
        
        int size = sourceCloud->size();
        sourceCloud->points[0].g = 255;
        sourceCloud->points[0].b = 255;
        sourceCloud->points[0].r = 0;
        sourceCloud->points[size-1].g = 255;
        sourceCloud->points[size-1].b = 90;
        sourceCloud->points[size-1].r = 0;
        sourceCloud->header.frame_id = "cloud";

        if(j==3){
            pub_test_.publish(sourceCloud);
        }

        
       // computing obstacle projection
        float theta_min, theta_max, obstacle_projection;
        theta_min = atan(sourceCloud->points[size-1].x / sourceCloud->points[size-1].y) * (180 / M_PI);
        theta_max = atan(sourceCloud->points[0].x / sourceCloud->points[0].y) * (180 / M_PI);
        
        
        // angle correction for Global Planning
        // if(sourceCloud->points[0].y<0){
        //     theta_min += 90;
        // }
        // else if(sourceCloud->points[0].y>0){
        //     theta_min += 270;
        // }
        

        // if(sourceCloud->points[size-1].y<0){
        //     theta_max += 90;
        // }
        // else if(sourceCloud->points[0].y>0){
        //     theta_max += 270;
        // }
        
        std::cout<<"\ntheta_min thata_max of "<<j<<" "<<theta_min<<" "<<theta_max<<std::endl;
        // computing obstacle projection
        obstacle_projection = fabs(theta_min-theta_max);
        
        // storing incoming min max according to their angular ranges
        if(theta_min>=222.5 and theta_min<=360){
        thetaMinArray_222to360.push_back(theta_min);
        }
        else if(theta_min>=0 and theta_min<=137.5){
        thetaMinArray_0to137.push_back(theta_min);
        }

        if(theta_max>=222.5 and theta_max<=360){
        thetaMaxArray_222to360.push_back(theta_max);
        }
        else if(theta_max>=0 and theta_max<=137.5){
        thetaMaxArray_0to137.push_back(theta_max);
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
    // pub_clusters_.publish(cloud_cluster);
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
    
    // sorting the vectors
    sort(thetaMinArray_222to360.begin(), thetaMinArray_222to360.end());
    sort(thetaMinArray_0to137.begin(), thetaMinArray_0to137.end());
    sort(thetaMaxArray_222to360.begin(), thetaMaxArray_222to360.end());
    sort(thetaMaxArray_0to137.begin(), thetaMaxArray_0to137.end());
    
    // concatenating the 2 vectors
    std::vector<float> thetaMinArray;
    std::vector<float> thetaMaxArray;

    thetaMinArray = concat(thetaMinArray_222to360, thetaMinArray_0to137);
    thetaMaxArray = concat(thetaMaxArray_222to360, thetaMaxArray_0to137);
    
    std::cout<<"\n``````````````````\n"; 
    for(int i=0; i<j; i++){
        std::cout<<std::endl<<"\nMin Angle "<<thetaMinArray[i];
        std::cout<<std::endl<<" Max Angle "<<thetaMaxArray[i];
    }
    std::cout<<"\n``````````````````\n"; 
    
    if(j!=0){
    clustering::decideFreeSector(thetaMinArray, thetaMaxArray, j);
    }

    thetaMaxArray.clear();
    thetaMinArray.clear();
   }
   pub_clusters_.publish(cloud_cluster);
}


void clustering::decideFreeSector(std::vector<float> thetaMin, std::vector<float> thetaMax, int numberOfClusters){


    rover::free_sectors data;

    std::vector<float>thetaMinFree;
    std::vector<float>thetaMaxFree;
    
    if(thetaMin[0]<222.5){
            thetaMin[0] = 222.5;
        }

    thetaMinFree.push_back(222.5);
    thetaMaxFree.push_back(thetaMin[0]);
    
       
    for(int i=0; i<numberOfClusters-1; i++){
        thetaMinFree.push_back(thetaMax[i]);
        thetaMaxFree.push_back(thetaMin[i+1]);
                
        }
    thetaMinFree.push_back(thetaMax[numberOfClusters-1]);
    thetaMaxFree.push_back(137.5);


        std::cout<<"\n~~~~~~~~~~~~~~~~~~\n"; 
        for(int i=0; i<numberOfClusters+1; i++){
        std::cout<<"\nTheta Min " <<thetaMinFree[i]<<" ";
        std::cout<<"Theta Max " <<thetaMaxFree[i]<<"\n";
        } 
        std::cout<<"\n~~~~~~~~~~~~~~~~~~\n";  
        std::cout<<"\n------------------\n";
    
    // finding cost aaray
    std::vector<float> costArray;
    for(int i=0; i<numberOfClusters+1; i++){
        float mid_angle_ = thetaMaxFree[i] + thetaMinFree[i];
        float cost = fabs(mid_angle_ - 60);
        costArray.push_back(cost);
    }
    
    // finding the appropriate free sector
    int index = 0, minimum = costArray[0];

    for(int i=1; i<numberOfClusters+1; i++){
        if(costArray[i] < minimum){
            minimum = costArray[i];
            index = i;
        }
    } 
    data.thetaMinFreeSector = thetaMinFree[index];
    data.thetaMaxFreeSector = thetaMaxFree[index];
    pub_free_sectors_.publish(data);

    std::cout<<"\nThe Appropriate free sector is "<<thetaMinFree[index]<<" "<<thetaMaxFree[index]<<std::endl;
   
  
    }




int main(int argc, char** argv){
    // initialising node
    ros::init(argc, argv, "pcl_cluster_2");
    ros::NodeHandle nh;
    clustering obj =  clustering(&nh);
    ros::spin();


}