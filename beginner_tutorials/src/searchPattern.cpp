#include "iostream"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Quaternion.h"
#include "math.h"
#include "time.h"
#include <string>


class SearchPattern{

 private:
    ros::Subscriber sub_imu,sub_id;
    ros::Publisher pub_vel;
    double yaw_set,yaw;
    ros::Time curr_time;
    int count;
    bool detected =0;
    bool reset_angle = 1;
    bool forward =0;
    bool shutupanddontturn = 0;
    geometry_msgs::Twist Vel;
    double elapsed_time;

  public:
   SearchPattern(ros::NodeHandle *nh){
//   sub_id = nh->subscribe("/marker_detect/id", 1000, &SearchPattern::id_callback, this);
   sub_imu = nh->subscribe("/imu", 10, &SearchPattern::imuCallback, this);
   pub_vel = nh->advertise<geometry_msgs::Twist> ("/cmd_vel", 10);
   }
   
   
/*void id_callback(const std_msgs::String::ConstPtr& msg_id)
{
    if(msg_id->data==" ")
    {
    detected =1;
    }
    else{
    detected = 0;
    }
    ROS_INFO_STREAM(msg_id->data);

}*/
   void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    yaw = msg->orientation.z;
    yaw = angle_correction(yaw);
    ROS_INFO_STREAM("yaw is "<<yaw);
    if(yaw>330 and yaw<361){
        Vel.angular.z = 1;//angle_diff/100;
    pub_vel.publish(Vel);
    reset_angle=1;
    }
    else{
    if(reset_angle)
    {
        set_angle(); 
    }
    else{
    
        rotation();
    }
    
    }
}


double angle_correction(double angle){
    if(count>=8){
    ros::shutdown();
    }
    double angle_conv;
    if(angle<0){
        return 360+angle;
    }
    else{
        return angle;
    }
}

void set_angle()
    {
        reset_angle = 0;
        if(yaw>330 and yaw<361){
        yaw_set = 0;
        shutupanddontturn=1;
        }
        
        else{
            yaw_set = yaw+30;
        }
        ROS_INFO_STREAM("yaw goal set to"<<yaw_set);
        
    }
    
void rotation(){
    ROS_INFO_STREAM("rotation called");
    double angle_diff = yaw_set - yaw;
    ROS_INFO_STREAM("Angle Difference"<<angle_diff);
    if(angle_diff>5)
    {
        Vel.angular.z = 0.75;
	sleep(2);//angle_diff/100;
        pub_vel.publish(Vel);
    }
    else{
        count = count +1;
        Vel.angular.z = 0;
        pub_vel.publish(Vel);
        forward=1;
	sleep(2);
        if(forward){
        move_forward();
        }
        
    }
    
    
    }
    
void move_forward(){
    ros::Time start_time = ros::Time::now();
    while(1){
        curr_time = ros::Time::now();
        elapsed_time = curr_time.toSec()  - start_time.toSec() ;
        if(elapsed_time<5){
            Vel.linear.x = 1;
	    sleep(2);
            pub_vel.publish(Vel);
        }
        if(elapsed_time>=5 and elapsed_time<=10){
            Vel.linear.x = -1;
	   sleep(2);
            pub_vel.publish(Vel);
        }
        if(elapsed_time>10){
            Vel.linear.x = 0;
	    sleep(2);
            pub_vel.publish(Vel);
            forward =0;
            reset_angle = 1;
            break;
        }

        ROS_INFO_STREAM("Elapsed Time since rotation : "<<elapsed_time);
    }
}
};



int main(int argc, char** argv)
{

  ros::init(argc, argv, "search_pattern");
  ros::NodeHandle nh;
  SearchPattern obj =  SearchPattern(&nh);
  ros::spin();
}
