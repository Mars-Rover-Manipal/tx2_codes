#include "arucoDetection.h"

using namespace ad;

arucoDetection::arucoDetection() {

    MarkerPose.header.frame_id = "zed2_base_link";

    ROS_INFO_STREAM("Started Aruco Detection Node");
    pub_marker = nh.advertise < sensor_msgs::Image > ("/marker_detect/image_raw", 100);					// Publisher for Image of detection
    pub_pose_estimated = nh.advertise < sensor_msgs::Image>("/marker_detect/estimated",100);				// Publisher for Image of estimation
    pub_marker_id = nh.advertise < std_msgs::String > ("/marker_detect/id", 100);						// Publisher for ID of detected marker
    pub_pose = nh.advertise < geometry_msgs::PoseStamped > ("marker/pose", 1000);						// Publisher for pose of detected marker
    pub_velocity = nh.advertise < geometry_msgs::Twist > ("/cmd_vel", 1);							// Publisher for rover velocity
    viz_marker_pub = nh.advertise < visualization_msgs::Marker >("/tag_marker",100);					// Publisher for Visual Marker of Tag
    sub_camera = nh.subscribe("/Zed2/image_raw", 100, & arucoDetection::imageCallback, this);			// Callback for Image from camera - Realsense d435
    //sub_camera_sim = nh.subscribe("/zed2/zed_node/rgb/image_rect_color", 100, &arucoDetection::imageCallback,this);		// General Callback for testing
}

void arucoDetection::imageCallback(const sensor_msgs::ImageConstPtr & msg) {
    ROS_INFO_STREAM("Got Image");
    camera_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    ROS_INFO_STREAM("Converted Image");
    arucoDetection::detect_aruco();
    //std::cout<<"Pose Estimated";
    msg_pub = camera_image -> toImageMsg();
    pub_marker.publish(msg_pub);
};

void arucoDetection::detect_aruco() {

    //set_params();
    
    markers_drawn_img = camera_image -> image;
    cv::aruco::detectMarkers(markers_drawn_img, dictionary, corners, marker_ids_detected, params);
    
    marker_ID.data = " ";
    
    for (int i = 0; i < marker_ids_detected.size(); i++) {
    
        ID = std::to_string(marker_ids_detected[i]);
        marker_ID.data.append(ID);
          
    }
    
    //pub_marker_id.publish(marker_ID);
    
    if (marker_ids_detected.size() > 0) {
    
        //cv::aruco::drawDetectedMarkers(markers_drawn_img, corners, marker_ids_detected);
    }
};


int main(int argc, char ** argv) {

    ros::init(argc, argv, "test_node");
    arucoDetection object = arucoDetection();
    ros::spin();
}
