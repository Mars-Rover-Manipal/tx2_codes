#include "pclProcessing.h"

using namespace pclProcessing;

clustering::clustering(ros::NodeHandle *nh){
        sub_lidar_ = nh->subscribe("/Lidar_3d", 1, &clustering::callback, this);
        pub_clusters_= nh->advertise<sensor_msgs::PointCloud2> ("/pcl_clustering", 1);
        pub_free_sectors_ = nh->advertise<rover::free_sectors>("/free_sectors", 1);
        pub_initiate_recovery_ = nh->advertise<std_msgs::Bool>("/intiate_recovery", 1);
    }

void clustering::callback(const sensor_msgs::PointCloud2ConstPtr& input){  

    // Pointcloud Containers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

    // vector for storing centroid and distance of each cluster
    std::vector<Eigen::Vector4f>CentroidArray;
    std::vector<float>DistanceArray;

    // vector for storing theta min or max and the associated centroid with it
    std::vector<thetaPlusCentroid>thetaMinArray_222to360;
    std::vector<thetaPlusCentroid>thetaMaxArray_222to360;
    std::vector<thetaPlusCentroid>thetaMinArray_0to137;
    std::vector<thetaPlusCentroid>thetaMaxArray_0to137;
  
    // converting pointcloud from what is suitable for ROS to what is suitable for PCL 
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
        
        // computing distance
        distance = sqrt(pow(Centroid.x(),2) + pow(Centroid.y(),2));
        CentroidArray.push_back(Centroid);
        DistanceArray.push_back(distance);

        
        int size = sourceCloud->size();
        
        // computing theta_min and theta_max
        float theta_min, theta_max, obstacle_projection;
        theta_min = atan(sourceCloud->points[size-1].x / sourceCloud->points[size-1].y) * (180 / M_PI);
        theta_max = atan(sourceCloud->points[0].x / sourceCloud->points[0].y) * (180 / M_PI);
        
        // angle correction for Global Planning

        // for theta_min
        if(sourceCloud->points[size-1].y<0){
            theta_min += 90;
        }
        else if(sourceCloud->points[size-1].y>0){
            theta_min += 270;
        }
        
        // for theta_max
        if(sourceCloud->points[0].y<0){
            theta_max += 90;
        }
        else if(sourceCloud->points[0].y>0){
            theta_max += 270;
        }
        

        std::cout<<"\ntheta_min thata_max "<<theta_min<<" "<<theta_max<<std::endl;
        
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
       
    } 
   
   // not to be executed when number of clusters = 0 or else Segmentation Fault occurs   
    if(j!=0){
    
        std::cout<<std::endl<<"Number of clusters: "<<j<<std::endl;

        // sorting vector of structures by the vslue of theta min and max
        sort(thetaMinArray_222to360.begin(), thetaMinArray_222to360.end(), compareByValue);
        sort(thetaMinArray_0to137.begin(), thetaMinArray_0to137.end(), compareByValue);
        sort(thetaMaxArray_222to360.begin(), thetaMaxArray_222to360.end(), compareByValue);
        sort(thetaMaxArray_0to137.begin(), thetaMaxArray_0to137.end(), compareByValue);
        
        // concatenating the 2 vectors
        std::vector<thetaPlusCentroid> thetaMinArray;
        std::vector<thetaPlusCentroid> thetaMaxArray;
        
        thetaMinArray = concat(thetaMinArray_222to360, thetaMinArray_0to137);
        thetaMaxArray = concat(thetaMaxArray_222to360, thetaMaxArray_0to137);
        
        std::cout<<"\n``````````````````\n"; 
        for(int i=0; i<j; i++){
            std::cout<<std::endl<<"\nMin Angle "<<thetaMinArray[i].theta;
            std::cout<<std::endl<<" Max Angle "<<thetaMaxArray[i].theta;
        //     if((thetaMinArray[i].theta>=222.5 and thetaMinArray[i].theta<=240) && (thetaMaxArray[i].theta>=120.0 and thetaMaxArray[i].theta<=137.5)){
        //         std_msgs::Bool recovery;
        //         recovery.data = 1;
        //         pub_initiate_recovery_.publish(recovery);
        // }
        }
        std::cout<<"\n``````````````````\n"; 
        
        // function for deciding free sectors
        if(j!=0){
            clustering::decideFreeSector(thetaMinArray, thetaMaxArray, j);
        }
        
        // emptying the vectors
        thetaMaxArray.clear();
        thetaMinArray.clear();
        thetaMinArray_222to360.clear();
        thetaMaxArray_222to360.clear();
        thetaMinArray_0to137.clear();
        thetaMaxArray_0to137.clear();
   }

   else if(j==0){

       rover::free_sectors data;
       data.thetaMaxFreeSector = 137.5;
       data.thetaMinFreeSector = 222.5;
       data.thetaMax_Centroid.x = 0;
       data.thetaMax_Centroid.y = 0;
       data.thetaMax_Centroid.z = 0;
       data.thetaMin_Centroid.x = 0;
       data.thetaMin_Centroid.y = 0;
       data.thetaMin_Centroid.z = 0;

       pub_free_sectors_.publish(data);
   }
}


void clustering::decideFreeSector(std::vector<thetaPlusCentroid> thetaMin, std::vector<thetaPlusCentroid> thetaMax, int numberOfClusters){
    
    rover::free_sectors data;
    float distance;
    
    // for storing angular ranges of free sectors in order
    std::vector<float>thetaMinFree;
    std::vector<float>thetaMaxFree;
    std::vector<float>distanceFromCentroid;
    std::vector<std::array<Eigen::Vector4f, 2> > Centroids;
    // temporary vector for storing centroid in the order (theta_min, theta_max)
    std::array<Eigen::Vector4f, 2> temp;
   
    
    // if(thetaMin[0].theta<222.5){
    //         thetaMin[0].theta = 222.5;
    //     }

    thetaMinFree.push_back(222.5);
    thetaMaxFree.push_back(thetaMin[0].theta);
    
    temp[0] = Eigen::Vector4f::Zero();
    temp[1] = thetaMin[0].Centroid;
    Centroids.push_back(temp);
    
    distance = sqrt(pow(thetaMin[0].Centroid.x(),2) + pow(thetaMin[0].Centroid.y(),2));
    
    distanceFromCentroid.push_back(distance);
       
    for(int i=0; i<numberOfClusters-1; i++){
        thetaMinFree.push_back(thetaMax[i].theta);
        thetaMaxFree.push_back(thetaMin[i+1].theta);

        temp[0] = thetaMax[i].Centroid;
        temp[1] = thetaMin[i+1].Centroid;
        Centroids.push_back(temp);

        distance = (sqrt(pow(thetaMax[i].Centroid.x(),2) + pow(thetaMax[i].Centroid.y(),2)) + sqrt(pow(thetaMin[i+1].Centroid.x(),2) + pow(thetaMin[i+1].Centroid.y(),2))) / 2;
        distanceFromCentroid.push_back(distance);
    }

    thetaMinFree.push_back(thetaMax[numberOfClusters-1].theta);
    thetaMaxFree.push_back(137.5);

    temp[0] = thetaMax[numberOfClusters-1].Centroid;
    temp[1] = Eigen::Vector4f::Zero();
    Centroids.push_back(temp);
    
    distance = sqrt(pow(thetaMax[numberOfClusters-1].Centroid.x(),2) + pow(thetaMax[numberOfClusters].Centroid.y(),2));
    distanceFromCentroid.push_back(distance);



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
    // std::vector<float> costArray;
    // for(int i=0; i<numberOfClusters+1; i++){

    //     // float mid_angle_ = (thetaMaxFree[i] + thetaMinFree[i])/2;
    //     // float cost = fabs(mid_angle_ - 300);

        
    //     float cost;
    //     costArray.push_back(cost);
    // }
    
    // finding the appropriate free sector
    int index = 0, minimum = distanceFromCentroid[0];
    
    for(int i=1; i<numberOfClusters+1; i++){
        if(distanceFromCentroid[i] < minimum){
            minimum = distanceFromCentroid[i];
            index = i;
        }
    } 
    
    // publishing the information regarding free sectors

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